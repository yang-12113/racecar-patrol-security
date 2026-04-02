import argparse
import os
from pathlib import Path

import cv2
import numpy as np


IMG_EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}


def l2_normalize(v):
    n = np.linalg.norm(v)
    if n < 1e-12:
        return v
    return v / n


def detect_largest_face_sface(detector, img):
    h, w = img.shape[:2]
    detector.setInputSize((w, h))
    _, faces = detector.detect(img)
    if faces is None or len(faces) == 0:
        return None
    areas = faces[:, 2] * faces[:, 3]
    return faces[int(np.argmax(areas))]


def extract_embedding_sface(detector, recognizer, img):
    face = detect_largest_face_sface(detector, img)
    if face is None:
        return None
    aligned = recognizer.alignCrop(img, face)
    feat = recognizer.feature(aligned).flatten().astype(np.float32)
    return l2_normalize(feat)


def extract_embedding_haar(cascade, hog, img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gray = cv2.equalizeHist(gray)
    faces = cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(40, 40))
    if len(faces) == 0:
        return None
    x, y, w, h = max(faces, key=lambda r: r[2] * r[3])
    face = gray[y : y + h, x : x + w]
    if face.size == 0:
        return None

    s1 = cv2.resize(face, (32, 32), interpolation=cv2.INTER_AREA).astype(np.float32).flatten() / 255.0
    s2_in = cv2.resize(face, (64, 64), interpolation=cv2.INTER_AREA)
    s2 = hog.compute(s2_in).flatten().astype(np.float32)
    s2 = l2_normalize(s2)
    feat = np.concatenate([s1, s2], axis=0).astype(np.float32)
    return l2_normalize(feat)


def choose_backend(det_model, rec_model, det_thres):
    if (
        det_model
        and rec_model
        and os.path.exists(det_model)
        and os.path.exists(rec_model)
        and os.path.getsize(det_model) > 0
        and os.path.getsize(rec_model) > 0
        and hasattr(cv2, "FaceDetectorYN_create")
        and hasattr(cv2, "FaceRecognizerSF_create")
    ):
        detector = cv2.FaceDetectorYN_create(det_model, "", (320, 320), det_thres, 0.3, 5000)
        recognizer = cv2.FaceRecognizerSF_create(rec_model, "")
        return "sface", detector, recognizer

    cascade = cv2.CascadeClassifier(cv2.data.haarcascades + "haarcascade_frontalface_default.xml")
    hog = cv2.HOGDescriptor(
        _winSize=(64, 64),
        _blockSize=(16, 16),
        _blockStride=(8, 8),
        _cellSize=(8, 8),
        _nbins=9,
    )
    return "haar", cascade, hog


def build_db(face_root, out_npz, det_model, rec_model, det_thres):
    face_root = Path(face_root)
    out_npz = Path(out_npz)
    out_npz.parent.mkdir(parents=True, exist_ok=True)

    method, detector, recognizer = choose_backend(det_model, rec_model, det_thres)
    print(f"[INFO] face backend: {method}")

    names = []
    feats = []
    counts = []

    total_imgs = 0
    ok_imgs = 0

    for person_dir in sorted(face_root.iterdir()):
        if not person_dir.is_dir():
            continue
        person_name = person_dir.name

        person_feats = []
        for p in sorted(person_dir.rglob("*")):
            if not p.is_file() or p.suffix.lower() not in IMG_EXTS:
                continue
            total_imgs += 1
            img = cv2.imread(str(p))
            if img is None:
                continue
            if method == "sface":
                feat = extract_embedding_sface(detector, recognizer, img)
            else:
                feat = extract_embedding_haar(detector, recognizer, img)
            if feat is None:
                continue
            person_feats.append(feat)
            ok_imgs += 1

        if len(person_feats) == 0:
            print(f"[WARN] {person_name}: no valid faces.")
            continue

        mean_feat = l2_normalize(np.mean(np.vstack(person_feats), axis=0).astype(np.float32))
        names.append(person_name)
        feats.append(mean_feat)
        counts.append(len(person_feats))
        print(f"[OK] {person_name}: {len(person_feats)} valid images.")

    if len(names) == 0:
        raise RuntimeError("No valid person embeddings generated.")

    np.savez(
        str(out_npz),
        names=np.asarray(names),
        feats=np.asarray(feats, dtype=np.float32),
        counts=np.asarray(counts, dtype=np.int32),
        metric=np.asarray(["cosine"], dtype=object),
        method=np.asarray([method], dtype=object),
    )
    print(f"[DONE] DB saved: {out_npz}")
    print(f"[STAT] persons={len(names)}, total_imgs={total_imgs}, valid_imgs={ok_imgs}")


def main():
    ap = argparse.ArgumentParser(description="Build face embedding DB from /root/face_db/raw/<name>/*.jpg")
    ap.add_argument("--face-root", default="/root/face_db/raw")
    ap.add_argument("--out", default="/root/face_db/embeddings.npz")
    ap.add_argument("--det-model", default="/root/face_models/face_detection_yunet_2023mar.onnx")
    ap.add_argument("--rec-model", default="/root/face_models/face_recognition_sface_2021dec.onnx")
    ap.add_argument("--det-thres", type=float, default=0.85)
    args = ap.parse_args()
    if not os.path.exists(args.face_root):
        raise FileNotFoundError(f"face root not found: {args.face_root}")

    build_db(args.face_root, args.out, args.det_model, args.rec_model, args.det_thres)


if __name__ == "__main__":
    main()
