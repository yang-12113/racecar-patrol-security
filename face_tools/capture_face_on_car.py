#!/usr/bin/env python3
import argparse
import os
import time
from pathlib import Path

import cv2


def make_detector(det_model, det_thres=0.85):
    if (
        det_model
        and os.path.exists(det_model)
        and os.path.getsize(det_model) > 0
        and hasattr(cv2, "FaceDetectorYN_create")
    ):
        detector = cv2.FaceDetectorYN_create(det_model, "", (320, 320), det_thres, 0.3, 5000)
        return "yunet", detector

    cascade = cv2.CascadeClassifier(cv2.data.haarcascades + "haarcascade_frontalface_default.xml")
    return "haar", cascade


def detect_faces(mode, detector, frame):
    if mode == "yunet":
        h, w = frame.shape[:2]
        detector.setInputSize((w, h))
        _, faces = detector.detect(frame)
        if faces is None or len(faces) == 0:
            return []
        out = []
        for row in faces:
            x, y, fw, fh = row[:4].astype(int).tolist()
            out.append((x, y, fw, fh))
        return out

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.equalizeHist(gray)
    faces = detector.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(50, 50))
    return list(faces)


def save_image(path, frame):
    ok, buf = cv2.imencode(".jpg", frame)
    if ok:
        buf.tofile(str(path))


def main():
    ap = argparse.ArgumentParser(description="Capture face samples directly from the car camera.")
    ap.add_argument("--name", default="owner")
    ap.add_argument("--num", type=int, default=20)
    ap.add_argument("--camera", type=int, default=0)
    ap.add_argument("--warmup", type=int, default=3)
    ap.add_argument("--every", type=int, default=6)
    ap.add_argument("--out-root", default="/root/face_db/raw")
    ap.add_argument("--det-model", default="/root/face_models/face_detection_yunet_2023mar.onnx")
    ap.add_argument("--show", action="store_true")
    args = ap.parse_args()

    person_dir = Path(args.out_root) / args.name
    person_dir.mkdir(parents=True, exist_ok=True)

    mode, detector = make_detector(args.det_model)
    print(f"[INFO] detector={mode}, out={person_dir}")

    cap = cv2.VideoCapture(args.camera, cv2.CAP_V4L2)
    if not cap.isOpened():
        cap.release()
        cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        raise RuntimeError("Cannot open car camera.")

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 30)

    end_t = time.time() + max(0, args.warmup)
    while time.time() < end_t:
        ok, frame = cap.read()
        if not ok:
            continue
        if args.show:
            cv2.putText(frame, "Get ready...", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
            cv2.imshow("capture_face_on_car", frame)
            if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                break

    saved = 0
    frame_id = 0
    while saved < max(1, args.num):
        ok, frame = cap.read()
        if not ok:
            continue

        faces = detect_faces(mode, detector, frame)
        if frame_id % max(1, args.every) == 0 and faces:
            ts = int(time.time() * 1000)
            out = person_dir / f"{args.name}_car_{saved:03d}_{ts}.jpg"
            save_image(out, frame)
            saved += 1

        if args.show:
            vis = frame.copy()
            for (x, y, w, h) in faces[:1]:
                cv2.rectangle(vis, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(
                vis,
                f"{args.name}: {saved}/{args.num}",
                (16, 32),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2,
            )
            cv2.imshow("capture_face_on_car", vis)
            if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                break

        frame_id += 1

    cap.release()
    cv2.destroyAllWindows()
    print(f"[DONE] saved={saved}, dir={person_dir}")


if __name__ == "__main__":
    main()
