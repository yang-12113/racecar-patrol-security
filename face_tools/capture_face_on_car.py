#!/usr/bin/env python3
import argparse
import json
import os
import time
from pathlib import Path

import cv2
import numpy as np


def l2_normalize(v):
    n = np.linalg.norm(v)
    if n < 1e-12:
        return v
    return v / n


def make_face_stack(det_model, rec_model, det_thres=0.85):
    recognizer = None
    if (
        det_model
        and os.path.exists(det_model)
        and os.path.getsize(det_model) > 0
        and hasattr(cv2, "FaceDetectorYN_create")
    ):
        detector = cv2.FaceDetectorYN_create(det_model, "", (320, 320), det_thres, 0.3, 5000)
        if (
            rec_model
            and os.path.exists(rec_model)
            and os.path.getsize(rec_model) > 0
            and hasattr(cv2, "FaceRecognizerSF_create")
        ):
            recognizer = cv2.FaceRecognizerSF_create(rec_model, "")
        return "yunet", detector, recognizer

    cascade = cv2.CascadeClassifier(cv2.data.haarcascades + "haarcascade_frontalface_default.xml")
    return "haar", cascade, None


def detect_faces(mode, detector, frame):
    if mode == "yunet":
        h, w = frame.shape[:2]
        detector.setInputSize((w, h))
        _, faces = detector.detect(frame)
        if faces is None or len(faces) == 0:
            return []
        out = []
        for row in faces:
            row = np.asarray(row, dtype=np.float32).flatten()
            x, y, fw, fh = row[:4].astype(int).tolist()
            score = float(row[14]) if row.shape[0] >= 15 else 0.0
            out.append(
                {
                    "bbox": [int(x), int(y), int(fw), int(fh)],
                    "score": score,
                    "row": row,
                }
            )
        out.sort(key=lambda item: item["bbox"][2] * item["bbox"][3], reverse=True)
        return out

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.equalizeHist(gray)
    faces = detector.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(50, 50))
    return [{"bbox": [int(x), int(y), int(w), int(h)], "score": 1.0, "row": None} for (x, y, w, h) in faces]


def clamp_bbox_xywh(x, y, w, h, frame_shape):
    fh, fw = frame_shape[:2]
    x = max(0, min(fw - 1, int(x)))
    y = max(0, min(fh - 1, int(y)))
    w = max(1, int(w))
    h = max(1, int(h))
    x2 = max(x + 1, min(fw, x + w))
    y2 = max(y + 1, min(fh, y + h))
    return x, y, x2, y2


def laplacian_sharpness(bgr):
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    return float(cv2.Laplacian(gray, cv2.CV_64F).var())


def gray_brightness(bgr):
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    return float(gray.mean())


def yunet_pose_score(row):
    if row is None or len(row) < 14:
        return 1.0
    re_x, re_y, le_x, le_y, nose_x, nose_y = [float(v) for v in row[4:10]]
    eye_dx = max(1.0, abs(le_x - re_x))
    eye_dy_ratio = abs(le_y - re_y) / eye_dx
    eye_mid_x = 0.5 * (le_x + re_x)
    nose_offset_ratio = abs(nose_x - eye_mid_x) / eye_dx
    raw = 1.0 - min(1.0, 0.65 * eye_dy_ratio + 0.35 * nose_offset_ratio)
    return max(0.0, min(1.0, raw))


def extract_sface_feature(recognizer, frame, face_row):
    if recognizer is None or face_row is None:
        return None
    try:
        aligned = recognizer.alignCrop(frame, face_row)
        feat = recognizer.feature(aligned).flatten().astype(np.float32)
        return l2_normalize(feat)
    except Exception:
        return None


def save_image(path, frame):
    ok, buf = cv2.imencode(".jpg", frame)
    if ok:
        buf.tofile(str(path))


def save_metadata(path, payload):
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")


def main():
    ap = argparse.ArgumentParser(description="Capture face samples directly from the car camera.")
    ap.add_argument("--name", default="owner")
    ap.add_argument("--num", type=int, default=20)
    ap.add_argument("--camera", type=int, default=0)
    ap.add_argument("--warmup", type=int, default=3)
    ap.add_argument("--every", type=int, default=6)
    ap.add_argument("--out-root", default="/root/face_db/raw")
    ap.add_argument("--det-model", default="/root/face_models/face_detection_yunet_2023mar.onnx")
    ap.add_argument("--rec-model", default="/root/face_models/face_recognition_sface_2021dec.onnx")
    ap.add_argument("--det-thres", type=float, default=0.85)
    ap.add_argument("--min-face-w", type=int, default=90)
    ap.add_argument("--min-face-h", type=int, default=110)
    ap.add_argument("--min-sharpness", type=float, default=80.0)
    ap.add_argument("--min-brightness", type=float, default=55.0)
    ap.add_argument("--max-brightness", type=float, default=210.0)
    ap.add_argument("--dedupe-sim", type=float, default=0.97)
    ap.add_argument("--show", action="store_true")
    args = ap.parse_args()

    person_dir = Path(args.out_root) / args.name
    person_dir.mkdir(parents=True, exist_ok=True)

    mode, detector, recognizer = make_face_stack(args.det_model, args.rec_model, det_thres=args.det_thres)
    print(f"[INFO] detector={mode}, recognizer={'sface' if recognizer is not None else 'none'}, out={person_dir}")

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
    last_saved_feat = None
    while saved < max(1, args.num):
        ok, frame = cap.read()
        if not ok:
            continue

        face_rows = detect_faces(mode, detector, frame)
        candidate = face_rows[0] if face_rows else None
        accepted = False
        reason = "no_face"
        meta = {}

        if candidate is not None and frame_id % max(1, args.every) == 0:
            x, y, fw, fh = candidate["bbox"]
            x1, y1, x2, y2 = clamp_bbox_xywh(x, y, fw, fh, frame.shape)
            face_crop = frame[y1:y2, x1:x2]
            sharpness = laplacian_sharpness(face_crop) if face_crop.size else 0.0
            brightness = gray_brightness(face_crop) if face_crop.size else 0.0
            pose_score = yunet_pose_score(candidate.get("row"))
            feat = extract_sface_feature(recognizer, frame, candidate.get("row"))

            reason = "accepted"
            if (x2 - x1) < int(args.min_face_w):
                reason = "small_w"
            elif (y2 - y1) < int(args.min_face_h):
                reason = "small_h"
            elif sharpness < float(args.min_sharpness):
                reason = "blurry"
            elif brightness < float(args.min_brightness):
                reason = "too_dark"
            elif brightness > float(args.max_brightness):
                reason = "too_bright"
            elif pose_score < 0.45:
                reason = "bad_pose"
            elif last_saved_feat is not None and feat is not None and float(np.dot(last_saved_feat, feat)) >= float(args.dedupe_sim):
                reason = "duplicate"
            else:
                accepted = True
                if feat is not None:
                    last_saved_feat = feat

            meta = {
                "name": str(args.name),
                "capture_ts": int(time.time() * 1000),
                "frame_id": int(frame_id),
                "face_bbox": [int(x1), int(y1), int(x2), int(y2)],
                "face_size": {"w": int(x2 - x1), "h": int(y2 - y1)},
                "detector_mode": str(mode),
                "detector_score": round(float(candidate.get("score", 0.0)), 6),
                "blur_score": round(float(sharpness), 4),
                "brightness": round(float(brightness), 4),
                "yaw_like_score": round(float(pose_score), 4),
                "accepted": bool(accepted),
                "reject_reason": "" if accepted else str(reason),
            }

            if accepted:
                ts = int(time.time() * 1000)
                out = person_dir / f"{args.name}_car_{saved:03d}_{ts}.jpg"
                save_image(out, frame)
                save_metadata(out.with_suffix(".json"), meta)
                saved += 1

        if args.show:
            vis = frame.copy()
            if candidate is not None:
                x, y, fw, fh = candidate["bbox"]
                x1, y1, x2, y2 = clamp_bbox_xywh(x, y, fw, fh, frame.shape)
                color = (0, 255, 0) if accepted else (0, 0, 255)
                cv2.rectangle(vis, (x1, y1), (x2, y2), color, 2)
                cv2.putText(
                    vis,
                    reason,
                    (x1, max(18, y1 - 10)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    color,
                    2,
                )
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
