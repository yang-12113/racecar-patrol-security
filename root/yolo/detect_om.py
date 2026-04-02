import cv2
import numpy as np
from acllite.acllite_model import AclLiteModel

INPUT_SIZE = 640
CONF_THRES = 0.4

def main():
    print("加载OM模型...")
    model = AclLiteModel("yolov5s.om")

    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("摄像头打开失败")
        return

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # ===== 预处理 =====
        img = cv2.resize(frame, (INPUT_SIZE, INPUT_SIZE))
        img = img.astype(np.float32) / 255.0
        img = img.transpose(2, 0, 1)
        img = np.expand_dims(img, axis=0)

        print("输入shape:", img.shape)

        # ===== 推理 =====
        outputs = model.execute([img])

        print("输出长度:", len(outputs))
        print("输出shape:", outputs[0].shape)

        if len(outputs) == 0 or outputs[0].size == 0:
            print("❌ 推理失败（空输出）")
            continue

        # ⚠️ 关键：先打印真实shape，不要瞎reshape
        pred = outputs[0]

        # ===== 如果是YOLOv5标准输出 =====
        try:
            pred = pred.reshape(-1, 85)
        except:
            print("⚠️ reshape失败，模型输出结构不同")
            continue

        # ===== 后处理 =====
        for det in pred:
            conf = det[4]
            if conf < CONF_THRES:
                continue

            cls_id = np.argmax(det[5:])
            score = conf * det[5 + cls_id]

            if score < CONF_THRES:
                continue

            x, y, w, h = det[:4]

            x1 = int(x - w / 2)
            y1 = int(y - h / 2)
            x2 = int(x + w / 2)
            y2 = int(y + h / 2)

            cv2.rectangle(frame, (x1, y1), (x2, y2), (0,255,0), 2)
            cv2.putText(frame, f"{score:.2f}", (x1, y1-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)

        cv2.imshow("OM YOLO", frame)

        if cv2.waitKey(1) == 27:
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
