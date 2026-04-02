import argparse
import json
import os
import shutil
import time
from datetime import datetime

import cv2
import numpy as np


def letterbox(im, new_shape=(640, 640), color=(114, 114, 114)):
    h, w = im.shape[:2]
    r = min(new_shape[0] / h, new_shape[1] / w)
    new_unpad = (int(round(w * r)), int(round(h * r)))
    dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]
    dw /= 2
    dh /= 2
    if (w, h) != new_unpad:
        im = cv2.resize(im, new_unpad, interpolation=cv2.INTER_LINEAR)
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    im = cv2.copyMakeBorder(im, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)
    return im, r, (dw, dh)


def load_backend(model, device_id):
    errs = []
    try:
        from ais_bench.infer.interface import InferSession

        sess = InferSession(device_id, model)
        return "ais_bench", sess
    except Exception as e:
        errs.append(f"ais_bench: {e}")

    try:
        import aclruntime

        try:
            sess = aclruntime.InferenceSession(model, device_id)
        except TypeError:
            sess = aclruntime.InferenceSession(model, device_id, 0)
        return "aclruntime", sess
    except Exception as e:
        errs.append(f"aclruntime: {e}")

    raise RuntimeError("No inference backend available. " + " | ".join(errs))


def run_infer(backend, sess, inp):
    if backend == "ais_bench":
        return sess.infer([inp])

    trials = []
    if hasattr(sess, "infer"):
        trials += [lambda: sess.infer([inp]), lambda: sess.infer(inp)]
    if hasattr(sess, "run"):
        trials += [lambda: sess.run([inp]), lambda: sess.run(None, [inp]), lambda: sess.run([inp], None)]

    last = None
    for f in trials:
        try:
            return f()
        except Exception as e:
            last = e
    raise RuntimeError(f"aclruntime infer failed: {last}")


def normalize_output(outputs):
    if isinstance(outputs, tuple):
        outputs = list(outputs)
    elif not isinstance(outputs, list):
        outputs = [outputs]

    mats = []
    for o in outputs:
        a = np.array(o)
        if a.size == 0:
            continue
        a = np.squeeze(a)
        if a.ndim == 2 and a.shape[1] >= 6:
            mats.append(a)
        elif a.ndim in (3, 4) and a.shape[-1] >= 6:
            mats.append(a.reshape(-1, a.shape[-1]))

    if not mats:
        raise RuntimeError("Cannot parse model output tensor shape.")

    pred = np.concatenate(mats, axis=0).astype(np.float32, copy=False)
    return pred


def postprocess(pred, ratio, dwdh, orig_shape, conf_thres=0.25, iou_thres=0.45, target_cls=-1):
    h0, w0 = orig_shape[:2]
    dw, dh = dwdh

    if pred.shape[1] >= 7:
        boxes = pred[:, :4]
        obj = pred[:, 4]
        cls_prob = pred[:, 5:]
        cls_id = np.argmax(cls_prob, axis=1)
        cls_conf = cls_prob[np.arange(cls_prob.shape[0]), cls_id]
        scores = obj * cls_conf

        keep = scores >= conf_thres
        boxes, scores, cls_id = boxes[keep], scores[keep], cls_id[keep]
        if len(boxes) == 0:
            return []

        xyxy = np.zeros_like(boxes)
        xyxy[:, 0] = boxes[:, 0] - boxes[:, 2] / 2
        xyxy[:, 1] = boxes[:, 1] - boxes[:, 3] / 2
        xyxy[:, 2] = boxes[:, 0] + boxes[:, 2] / 2
        xyxy[:, 3] = boxes[:, 1] + boxes[:, 3] / 2
    else:
        xyxy = pred[:, :4]
        scores = pred[:, 4]
        cls_id = pred[:, 5].astype(np.int32)

        keep = scores >= conf_thres
        xyxy, scores, cls_id = xyxy[keep], scores[keep], cls_id[keep]
        if len(xyxy) == 0:
            return []

    xyxy[:, [0, 2]] = (xyxy[:, [0, 2]] - dw) / ratio
    xyxy[:, [1, 3]] = (xyxy[:, [1, 3]] - dh) / ratio

    xyxy[:, 0] = np.clip(xyxy[:, 0], 0, w0 - 1)
    xyxy[:, 1] = np.clip(xyxy[:, 1], 0, h0 - 1)
    xyxy[:, 2] = np.clip(xyxy[:, 2], 0, w0 - 1)
    xyxy[:, 3] = np.clip(xyxy[:, 3], 0, h0 - 1)

    nms_boxes = []
    for b in xyxy:
        x1, y1, x2, y2 = b.tolist()
        nms_boxes.append([x1, y1, max(0.0, x2 - x1), max(0.0, y2 - y1)])

    idxs = cv2.dnn.NMSBoxes(nms_boxes, scores.tolist(), conf_thres, iou_thres)
    if len(idxs) == 0:
        return []

    idxs = np.array(idxs).reshape(-1)
    out = []
    for i in idxs:
        x1, y1, x2, y2 = xyxy[i].tolist()
        cls = int(cls_id[i])
        if target_cls >= 0 and cls != target_cls:
            continue
        out.append({
            "cls": cls,
            "conf": float(scores[i]),
            "bbox": [int(x1), int(y1), int(x2), int(y2)],
        })
    return out


def make_temp_writer(width, height, fps, record_step=1):
    base = datetime.now().strftime("tmp_rec_%Y%m%d_%H%M%S")
    mp4_path = f"/tmp/{base}.mp4"
    avi_path = f"/tmp/{base}.avi"

    save_fps = fps / max(1, record_step)
    if save_fps < 1:
        save_fps = 1

    writer = cv2.VideoWriter(mp4_path, cv2.VideoWriter_fourcc(*"mp4v"), save_fps, (width, height))
    if writer.isOpened():
        return writer, mp4_path

    writer.release()
    writer = cv2.VideoWriter(avi_path, cv2.VideoWriter_fourcc(*"XVID"), save_fps, (width, height))
    if writer.isOpened():
        return writer, avi_path

    writer.release()
    raise RuntimeError("Failed to create temporary video writer.")


def unique_path(path):
    if not os.path.exists(path):
        return path
    stem, ext = os.path.splitext(path)
    idx = 1
    while True:
        p = f"{stem}_{idx}{ext}"
        if not os.path.exists(p):
            return p
        idx += 1


def ask_save_recording(tmp_video):
    if not tmp_video or not os.path.exists(tmp_video):
        print("[Recorder] No temporary video found.")
        return

    try:
        choice = input("\n是否保存本次录像? [y/N]: ").strip().lower()
    except EOFError:
        choice = "n"

    if choice not in ("y", "yes"):
        try:
            os.remove(tmp_video)
            print(f"[Recorder] 已丢弃临时录像: {tmp_video}")
        except OSError:
            pass
        return

    default_dir = "/root/runs/videos"
    try:
        save_dir = input(f"保存目录(默认 {default_dir}): ").strip()
    except EOFError:
        save_dir = ""
    if not save_dir:
        save_dir = default_dir

    os.makedirs(save_dir, exist_ok=True)

    ts = datetime.now().strftime("cam_%Y%m%d_%H%M%S")
    ext = os.path.splitext(tmp_video)[1] or ".mp4"
    try:
        name = input(f"文件名(不含扩展, 默认 {ts}): ").strip()
    except EOFError:
        name = ""
    if not name:
        name = ts

    if name.lower().endswith(ext.lower()):
        final_path = os.path.join(save_dir, name)
    else:
        final_path = os.path.join(save_dir, name + ext)

    final_path = unique_path(final_path)
    shutil.move(tmp_video, final_path)
    print(f"[Recorder] 录像已保存: {final_path}")


def main():
    ap = argparse.ArgumentParser(
        description="Ascend OM camera inference (one-click run, ask save video on exit)."
    )
    ap.add_argument("--model", default="/root/yolov5s_310b.om")
    ap.add_argument("--device", type=int, default=0)
    ap.add_argument("--camera", type=int, default=0)
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--conf", type=float, default=0.35)
    ap.add_argument("--iou", type=float, default=0.45)
    ap.add_argument("--target-cls", type=int, default=0, help="-1保留所有类别，0仅人")
    ap.add_argument("--infer-interval", type=int, default=2, help="每N帧推理一次，其他帧复用结果")
    ap.add_argument("--print-every", type=int, default=10, help="每N帧打印一次结果，0表示不打印")
    ap.add_argument("--record-step", type=int, default=2, help="录像每N帧写入一次，减小IO开销")
    ap.add_argument("--no-show", action="store_true")
    args = ap.parse_args()

    args.infer_interval = max(1, args.infer_interval)
    args.record_step = max(1, args.record_step)

    backend, sess = load_backend(args.model, args.device)
    print(
        f"backend={backend}, model={args.model}, infer_interval={args.infer_interval}, "
        f"record_step={args.record_step}, target_cls={args.target_cls}"
    )
    print("按 q 或 ESC 结束。结束后会询问是否保存录像。")

    cap = cv2.VideoCapture(args.camera, cv2.CAP_V4L2)
    if not cap.isOpened():
        cap.release()
        cap = cv2.VideoCapture(args.camera)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if not cap.isOpened():
        raise RuntimeError("Cannot open camera.")

    fps = cap.get(cv2.CAP_PROP_FPS)
    if not fps or fps < 1:
        fps = 20.0

    writer = None
    tmp_video = None

    fid = 0
    last_dets = []
    last_infer_ms = 0.0
    while True:
        ok, frame = cap.read()
        if not ok:
            break

        if writer is None:
            h, w = frame.shape[:2]
            writer, tmp_video = make_temp_writer(w, h, fps, args.record_step)
            print(f"[Recorder] 临时录制中: {tmp_video}")

        run_this_frame = (fid % args.infer_interval == 0) or (fid == 0)
        if run_this_frame:
            img, ratio, dwdh = letterbox(frame, (args.imgsz, args.imgsz))
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
            inp = np.transpose(img, (2, 0, 1))[None, ...]
            inp = np.ascontiguousarray(inp)

            t0 = time.time()
            raw = run_infer(backend, sess, inp)
            pred = normalize_output(raw)
            dets = postprocess(pred, ratio, dwdh, frame.shape, args.conf, args.iou, args.target_cls)
            dt = (time.time() - t0) * 1000.0

            last_dets = dets
            last_infer_ms = dt
        else:
            dets = last_dets
            dt = last_infer_ms

        if args.print_every > 0 and fid % args.print_every == 0:
            print(json.dumps({"frame": fid, "infer_ms": round(dt, 2), "bbox": dets}, ensure_ascii=False))

        for d in dets:
            x1, y1, x2, y2 = d["bbox"]
            txt = f'{d["cls"]}:{d["conf"]:.2f}'
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, txt, (x1, max(0, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)

        if fid % args.record_step == 0:
            writer.write(frame)

        if not args.no_show:
            cv2.imshow("ascend_yolo_bbox", frame)
            key = cv2.waitKey(1) & 0xFF
            if key == 27 or key == ord("q"):
                break

        fid += 1

    cap.release()
    if writer is not None:
        writer.release()
    cv2.destroyAllWindows()

    ask_save_recording(tmp_video)


if __name__ == "__main__":
    main()
