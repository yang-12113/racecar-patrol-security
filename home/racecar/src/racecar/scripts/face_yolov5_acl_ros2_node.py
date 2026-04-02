#!/usr/bin/env python3
import os
import time
from typing import List, Tuple

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray

try:
    import acl
except Exception:
    acl = None


class ACLYoloRunner:
    def __init__(self, model_path: str, device_id: int = 0):
        if acl is None:
            raise RuntimeError("python ACL package not found. Install CANN runtime/python ACL first.")
        if not os.path.isfile(model_path):
            raise FileNotFoundError(f"OM model not found: {model_path}")

        self.model_path = model_path
        self.device_id = device_id
        self.model_id = None
        self.model_desc = None
        self.context = None
        self.stream = None
        self.input_dataset = None
        self.output_dataset = None
        self.input_buffers = []
        self.output_buffers = []

        self._init_acl()
        self._load_model()
        self._alloc_buffers()

    @staticmethod
    def _check(ret: int, msg: str):
        if ret != 0:
            raise RuntimeError(f"{msg}, ret={ret}")

    def _init_acl(self):
        ret = acl.init()
        if ret not in (0, 100002):
            self._check(ret, "acl.init failed")
        self._check(acl.rt.set_device(self.device_id), "acl.rt.set_device failed")
        self.context, ret = acl.rt.create_context(self.device_id)
        self._check(ret, "acl.rt.create_context failed")
        self.stream, ret = acl.rt.create_stream()
        self._check(ret, "acl.rt.create_stream failed")

    def _load_model(self):
        self.model_id, ret = acl.mdl.load_from_file(self.model_path)
        self._check(ret, "acl.mdl.load_from_file failed")

        self.model_desc = acl.mdl.create_desc()
        self._check(acl.mdl.get_desc(self.model_desc, self.model_id), "acl.mdl.get_desc failed")

    def _alloc_buffers(self):
        input_num = acl.mdl.get_num_inputs(self.model_desc)
        output_num = acl.mdl.get_num_outputs(self.model_desc)

        self.input_dataset = acl.mdl.create_dataset()
        for i in range(input_num):
            size = acl.mdl.get_input_size_by_index(self.model_desc, i)
            dev_ptr, ret = acl.rt.malloc(size, acl.ACL_MEM_MALLOC_HUGE_FIRST)
            self._check(ret, f"acl.rt.malloc input[{i}] failed")
            data_buf = acl.create_data_buffer(dev_ptr, size)
            self._check(acl.mdl.add_dataset_buffer(self.input_dataset, data_buf), f"add input dataset[{i}] failed")
            self.input_buffers.append((dev_ptr, size, data_buf))

        self.output_dataset = acl.mdl.create_dataset()
        for i in range(output_num):
            size = acl.mdl.get_output_size_by_index(self.model_desc, i)
            dev_ptr, ret = acl.rt.malloc(size, acl.ACL_MEM_MALLOC_HUGE_FIRST)
            self._check(ret, f"acl.rt.malloc output[{i}] failed")
            data_buf = acl.create_data_buffer(dev_ptr, size)
            self._check(acl.mdl.add_dataset_buffer(self.output_dataset, data_buf), f"add output dataset[{i}] failed")
            self.output_buffers.append((dev_ptr, size, data_buf))

    def infer(self, input_tensor: np.ndarray) -> List[np.ndarray]:
        if input_tensor.dtype != np.float32:
            input_tensor = input_tensor.astype(np.float32)
        input_tensor = np.ascontiguousarray(input_tensor)

        input_ptr = acl.util.numpy_to_ptr(input_tensor)
        in_dev_ptr, in_size, _ = self.input_buffers[0]
        if input_tensor.nbytes > in_size:
            raise RuntimeError(f"Input bytes {input_tensor.nbytes} > model input buffer {in_size}")

        self._check(
            acl.rt.memcpy(in_dev_ptr, in_size, input_ptr, input_tensor.nbytes, acl.ACL_MEMCPY_HOST_TO_DEVICE),
            "H2D memcpy failed",
        )

        self._check(acl.mdl.execute(self.model_id, self.input_dataset, self.output_dataset), "acl.mdl.execute failed")

        outputs = []
        for i, (out_dev_ptr, out_size, _) in enumerate(self.output_buffers):
            out_host = np.zeros((out_size,), dtype=np.uint8)
            out_host_ptr = acl.util.numpy_to_ptr(out_host)
            self._check(
                acl.rt.memcpy(out_host_ptr, out_size, out_dev_ptr, out_size, acl.ACL_MEMCPY_DEVICE_TO_HOST),
                f"D2H memcpy output[{i}] failed",
            )
            out_f32 = np.frombuffer(out_host.tobytes(), dtype=np.float32).copy()
            outputs.append(out_f32)

        return outputs

    def release(self):
        for dev_ptr, _, data_buf in self.input_buffers:
            acl.destroy_data_buffer(data_buf)
            acl.rt.free(dev_ptr)
        for dev_ptr, _, data_buf in self.output_buffers:
            acl.destroy_data_buffer(data_buf)
            acl.rt.free(dev_ptr)

        if self.input_dataset is not None:
            acl.mdl.destroy_dataset(self.input_dataset)
        if self.output_dataset is not None:
            acl.mdl.destroy_dataset(self.output_dataset)

        if self.model_id is not None:
            acl.mdl.unload(self.model_id)
        if self.model_desc is not None:
            acl.mdl.destroy_desc(self.model_desc)

        if self.stream is not None:
            acl.rt.destroy_stream(self.stream)
        if self.context is not None:
            acl.rt.destroy_context(self.context)

        acl.rt.reset_device(self.device_id)
        acl.finalize()


class FaceYoloAclNode(Node):
    def __init__(self):
        super().__init__("face_yolov5_acl_node")

        self.declare_parameter("model_path", "/root/yolov5_face.om")
        self.declare_parameter("device_id", 0)
        self.declare_parameter("img_w", 640)
        self.declare_parameter("img_h", 640)
        self.declare_parameter("conf_thres", 0.35)
        self.declare_parameter("iou_thres", 0.45)
        self.declare_parameter("max_det", 20)

        model_path = self.get_parameter("model_path").get_parameter_value().string_value
        device_id = self.get_parameter("device_id").get_parameter_value().integer_value
        self.img_w = int(self.get_parameter("img_w").get_parameter_value().integer_value)
        self.img_h = int(self.get_parameter("img_h").get_parameter_value().integer_value)
        self.conf_thres = float(self.get_parameter("conf_thres").get_parameter_value().double_value)
        self.iou_thres = float(self.get_parameter("iou_thres").get_parameter_value().double_value)
        self.max_det = int(self.get_parameter("max_det").get_parameter_value().integer_value)

        self.pub = self.create_publisher(Float32MultiArray, "/yolo_bbox", 10)
        self.sub = self.create_subscription(Image, "/camera/image_raw", self.image_cb, 1)

        self.runner = ACLYoloRunner(model_path=model_path, device_id=int(device_id))
        self.last_log_t = 0.0

        self.get_logger().info(f"Node started. model={model_path}, sub=/camera/image_raw, pub=/yolo_bbox")

    @staticmethod
    def rosimg_to_bgr(msg: Image) -> np.ndarray:
        if msg.encoding not in ("bgr8", "rgb8"):
            raise ValueError(f"Unsupported encoding: {msg.encoding}")

        channels = 3
        row = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
        img = row[:, : msg.width * channels].reshape(msg.height, msg.width, channels)
        if msg.encoding == "rgb8":
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        return img

    @staticmethod
    def letterbox(im: np.ndarray, new_shape: Tuple[int, int] = (640, 640), color=(114, 114, 114)):
        h, w = im.shape[:2]
        r = min(new_shape[0] / h, new_shape[1] / w)
        new_unpad = (int(round(w * r)), int(round(h * r)))
        dw = new_shape[1] - new_unpad[0]
        dh = new_shape[0] - new_unpad[1]
        dw /= 2
        dh /= 2

        if (w, h) != new_unpad:
            im = cv2.resize(im, new_unpad, interpolation=cv2.INTER_LINEAR)

        top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
        left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
        im = cv2.copyMakeBorder(im, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)
        return im, r, (dw, dh)

    @staticmethod
    def xywh2xyxy(x: np.ndarray) -> np.ndarray:
        y = x.copy()
        y[:, 0] = x[:, 0] - x[:, 2] / 2
        y[:, 1] = x[:, 1] - x[:, 3] / 2
        y[:, 2] = x[:, 0] + x[:, 2] / 2
        y[:, 3] = x[:, 1] + x[:, 3] / 2
        return y

    @staticmethod
    def nms(boxes: np.ndarray, scores: np.ndarray, iou_thres: float, max_det: int) -> List[int]:
        if boxes.shape[0] == 0:
            return []

        x1 = boxes[:, 0]
        y1 = boxes[:, 1]
        x2 = boxes[:, 2]
        y2 = boxes[:, 3]

        areas = (x2 - x1).clip(0) * (y2 - y1).clip(0)
        order = scores.argsort()[::-1]
        keep = []

        while order.size > 0 and len(keep) < max_det:
            i = order[0]
            keep.append(int(i))
            if order.size == 1:
                break

            xx1 = np.maximum(x1[i], x1[order[1:]])
            yy1 = np.maximum(y1[i], y1[order[1:]])
            xx2 = np.minimum(x2[i], x2[order[1:]])
            yy2 = np.minimum(y2[i], y2[order[1:]])

            w = np.maximum(0.0, xx2 - xx1)
            h = np.maximum(0.0, yy2 - yy1)
            inter = w * h
            iou = inter / (areas[i] + areas[order[1:]] - inter + 1e-6)

            order = order[1:][iou < iou_thres]

        return keep

    def decode_yolo(self, raw: np.ndarray, img0_shape: Tuple[int, int], ratio: float, dwdh: Tuple[float, float]):
        if raw.size == 0:
            return np.zeros((0, 6), dtype=np.float32)

        preds = None
        if raw.size % 16 == 0:
            p = raw.reshape(-1, 16)
            boxes = self.xywh2xyxy(p[:, :4])
            scores = p[:, 4] * p[:, 15]
            classes = np.zeros_like(scores)
            preds = np.stack([boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3], scores, classes], axis=1)
        elif raw.size % 6 == 0:
            preds = raw.reshape(-1, 6)
        elif raw.size % 85 == 0:
            p = raw.reshape(-1, 85)
            boxes = self.xywh2xyxy(p[:, :4])
            class_scores = p[:, 5:]
            cls = np.argmax(class_scores, axis=1)
            cls_conf = class_scores[np.arange(class_scores.shape[0]), cls]
            scores = p[:, 4] * cls_conf
            preds = np.concatenate([boxes, scores[:, None], cls[:, None].astype(np.float32)], axis=1)
        else:
            return np.zeros((0, 6), dtype=np.float32)

        preds = preds[preds[:, 4] >= self.conf_thres]
        if preds.shape[0] == 0:
            return preds

        dw, dh = dwdh
        preds[:, [0, 2]] -= dw
        preds[:, [1, 3]] -= dh
        preds[:, :4] /= ratio

        h0, w0 = img0_shape
        preds[:, 0] = np.clip(preds[:, 0], 0, w0 - 1)
        preds[:, 1] = np.clip(preds[:, 1], 0, h0 - 1)
        preds[:, 2] = np.clip(preds[:, 2], 0, w0 - 1)
        preds[:, 3] = np.clip(preds[:, 3], 0, h0 - 1)

        keep = self.nms(preds[:, :4], preds[:, 4], self.iou_thres, self.max_det)
        return preds[keep]

    def image_cb(self, msg: Image):
        try:
            bgr = self.rosimg_to_bgr(msg)
            img0_h, img0_w = bgr.shape[:2]

            img, ratio, dwdh = self.letterbox(bgr, (self.img_h, self.img_w))
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            img = img.astype(np.float32) / 255.0
            img = np.transpose(img, (2, 0, 1))[None, :, :, :]

            outputs = self.runner.infer(img)
            det = self.decode_yolo(outputs[0], (img0_h, img0_w), ratio, dwdh)

            out = Float32MultiArray()
            if det.shape[0] > 0:
                out.data = det.astype(np.float32).reshape(-1).tolist()
            else:
                out.data = []
            self.pub.publish(out)

            now = time.time()
            if now - self.last_log_t > 1.0:
                self.get_logger().info(f"det={det.shape[0]} image={msg.width}x{msg.height}")
                self.last_log_t = now

        except Exception as e:
            self.get_logger().error(f"inference failed: {e}")

    def destroy_node(self):
        try:
            if hasattr(self, "runner") and self.runner is not None:
                self.runner.release()
        except Exception as e:
            self.get_logger().warn(f"release warning: {e}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = FaceYoloAclNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
