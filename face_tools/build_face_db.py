import argparse
import os
from pathlib import Path

import cv2
import numpy as np


IMG_EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}
BUILD_VERSION = "face_db_v2"


def l2_normalize(v):
    n = np.linalg.norm(v)
    if n < 1e-12:
        return v
    return v / n


def clamp01(v):
    return max(0.0, min(1.0, float(v)))


def detect_largest_face_sface(detector, img):
    h, w = img.shape[:2]
    detector.setInputSize((w, h))
    _, faces = detector.detect(img)
    if faces is None or len(faces) == 0:
        return None
    areas = faces[:, 2] * faces[:, 3]
    return faces[int(np.argmax(areas))]


def laplacian_sharpness(bgr):
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    return float(cv2.Laplacian(gray, cv2.CV_64F).var())


def gray_brightness(bgr):
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    return float(gray.mean())


def face_quality_score(width, height, frame_shape, sharpness, brightness, det_score=1.0):
    fh, fw = frame_shape[:2]
    area_ratio = float(width * height) / max(1.0, float(fw * fh))
    size_score = clamp01((area_ratio - 0.02) / 0.10)
    sharp_score = clamp01(float(sharpness) / 180.0)
    if brightness < 55.0:
        bright_score = clamp01(brightness / 55.0)
    elif brightness > 210.0:
        bright_score = clamp01((255.0 - brightness) / 45.0)
    else:
        bright_score = 1.0
    det_score = clamp01(det_score)
    return clamp01(0.35 * size_score + 0.30 * sharp_score + 0.20 * bright_score + 0.15 * det_score)


def extract_embedding_sface(detector, recognizer, img):
    face = detect_largest_face_sface(detector, img)
    if face is None:
        return None, None
    aligned = recognizer.alignCrop(img, face)
    feat = recognizer.feature(aligned).flatten().astype(np.float32)
    return l2_normalize(feat), face


def extract_embedding_haar(cascade, hog, img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gray = cv2.equalizeHist(gray)
    faces = cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(40, 40))
    if len(faces) == 0:
        return None, None
    x, y, w, h = max(faces, key=lambda r: r[2] * r[3])
    face = gray[y : y + h, x : x + w]
    if face.size == 0:
        return None, None

    s1 = cv2.resize(face, (32, 32), interpolation=cv2.INTER_AREA).astype(np.float32).flatten() / 255.0
    s2_in = cv2.resize(face, (64, 64), interpolation=cv2.INTER_AREA)
    s2 = hog.compute(s2_in).flatten().astype(np.float32)
    s2 = l2_normalize(s2)
    feat = np.concatenate([s1, s2], axis=0).astype(np.float32)
    return l2_normalize(feat), np.asarray([x, y, w, h], dtype=np.float32)


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


def summarize_support_scores(feats):
    feats = np.asarray(feats, dtype=np.float32)
    if len(feats) == 0:
        return np.zeros((0,), dtype=np.float32), np.zeros((0,), dtype=np.float32)
    sims = feats @ feats.T
    np.fill_diagonal(sims, -1.0)
    nn = np.max(sims, axis=1) if len(feats) > 1 else np.ones((1,), dtype=np.float32)
    topk_mean = []
    for row in sims:
        valid = row[row > -0.5]
        if valid.size == 0:
            topk_mean.append(1.0)
            continue
        topk = np.sort(valid)[::-1][: min(3, valid.size)]
        topk_mean.append(float(np.mean(topk)))
    return nn.astype(np.float32), np.asarray(topk_mean, dtype=np.float32)


def select_kept_indices(records, aggregate, outlier_drop_ratio, topk):
    if not records:
        return []
    if aggregate in ("all", "mean"):
        return list(range(len(records)))
    if len(records) <= 3:
        return list(range(len(records)))

    quality_floor = 0.35
    base_keep = [idx for idx, row in enumerate(records) if float(row["quality_score"]) >= quality_floor]
    if len(base_keep) < max(3, min(len(records), 4)):
        base_keep = list(range(len(records)))

    kept_records = [records[idx] for idx in base_keep]
    feats = np.vstack([row["feat"] for row in kept_records]).astype(np.float32)
    nn, topk_mean = summarize_support_scores(feats)
    support = 0.6 * topk_mean + 0.4 * nn

    ranked = []
    for local_idx, base_idx in enumerate(base_keep):
        quality = float(records[base_idx]["quality_score"])
        ranked.append((0.7 * float(support[local_idx]) + 0.3 * quality, base_idx))
    ranked.sort(reverse=True, key=lambda item: item[0])

    drop_count = 0
    if len(ranked) > 4:
        drop_count = min(int(round(len(ranked) * float(outlier_drop_ratio))), max(0, len(ranked) - 3))
    ranked = ranked[: len(ranked) - drop_count] if drop_count > 0 else ranked

    if aggregate == "topk":
        keep_count = min(max(1, int(topk)), len(ranked))
        ranked = ranked[:keep_count]

    kept = [base_idx for _, base_idx in ranked]
    kept.sort()
    return kept


def build_db(face_root, out_npz, det_model, rec_model, det_thres, aggregate, outlier_drop_ratio, topk):
    face_root = Path(face_root)
    out_npz = Path(out_npz)
    out_npz.parent.mkdir(parents=True, exist_ok=True)

    method, detector, recognizer = choose_backend(det_model, rec_model, det_thres)
    print(f"[INFO] face backend: {method}")

    final_names = []
    final_feats = []
    final_quality_scores = []
    final_sample_paths = []
    counts = []

    all_sample_paths = []
    all_sample_names = []
    all_quality_scores = []
    pruned_mask = []

    total_imgs = 0
    ok_imgs = 0

    for person_dir in sorted(face_root.iterdir()):
        if not person_dir.is_dir():
            continue
        person_name = person_dir.name
        records = []

        for p in sorted(person_dir.rglob("*")):
            if not p.is_file() or p.suffix.lower() not in IMG_EXTS:
                continue
            total_imgs += 1
            img = cv2.imread(str(p))
            if img is None:
                continue

            if method == "sface":
                feat, face = extract_embedding_sface(detector, recognizer, img)
            else:
                feat, face = extract_embedding_haar(detector, recognizer, img)
            if feat is None or face is None:
                continue

            if method == "sface":
                x, y, w, h = [float(v) for v in face[:4]]
                det_score = float(face[14]) if len(face) >= 15 else 1.0
            else:
                x, y, w, h = [float(v) for v in face[:4]]
                det_score = 1.0
            x1 = max(0, int(x))
            y1 = max(0, int(y))
            x2 = min(img.shape[1], max(x1 + 1, int(round(x + w))))
            y2 = min(img.shape[0], max(y1 + 1, int(round(y + h))))
            crop = img[y1:y2, x1:x2]
            sharpness = laplacian_sharpness(crop) if crop.size else 0.0
            brightness = gray_brightness(crop) if crop.size else 0.0
            quality = face_quality_score(x2 - x1, y2 - y1, img.shape, sharpness, brightness, det_score)

            records.append(
                {
                    "path": str(p),
                    "feat": feat,
                    "quality_score": float(quality),
                    "bbox": [int(x1), int(y1), int(x2), int(y2)],
                    "sharpness": float(sharpness),
                    "brightness": float(brightness),
                    "detector_score": float(det_score),
                }
            )
            ok_imgs += 1

        if len(records) == 0:
            print(f"[WARN] {person_name}: no valid faces.")
            continue

        kept_indices = select_kept_indices(records, aggregate, outlier_drop_ratio, topk)
        if not kept_indices:
            print(f"[WARN] {person_name}: all samples were rejected by pruning, keeping raw records.")
            kept_indices = list(range(len(records)))

        for idx, row in enumerate(records):
            all_sample_paths.append(str(row["path"]))
            all_sample_names.append(str(person_name))
            all_quality_scores.append(float(row["quality_score"]))
            pruned_mask.append(1 if idx in kept_indices else 0)

        kept_records = [records[idx] for idx in kept_indices]
        counts.append(len(kept_records))

        if aggregate in ("mean", "mean_pruned"):
            mean_feat = l2_normalize(np.mean(np.vstack([row["feat"] for row in kept_records]), axis=0).astype(np.float32))
            final_names.append(person_name)
            final_feats.append(mean_feat)
            final_quality_scores.append(float(np.mean([row["quality_score"] for row in kept_records])))
            final_sample_paths.append(str(kept_records[0]["path"]))
            print(
                f"[OK] {person_name}: valid={len(records)} kept={len(kept_records)} -> 1 {aggregate} embedding."
            )
        else:
            for row in kept_records:
                final_names.append(person_name)
                final_feats.append(row["feat"])
                final_quality_scores.append(float(row["quality_score"]))
                final_sample_paths.append(str(row["path"]))
            print(
                f"[OK] {person_name}: valid={len(records)} kept={len(kept_records)} aggregate={aggregate}."
            )

    if len(final_names) == 0:
        raise RuntimeError("No valid person embeddings generated.")

    np.savez(
        str(out_npz),
        names=np.asarray(final_names),
        feats=np.asarray(final_feats, dtype=np.float32),
        counts=np.asarray(counts, dtype=np.int32),
        metric=np.asarray(["cosine"], dtype=object),
        method=np.asarray([method], dtype=object),
        aggregate=np.asarray([aggregate], dtype=object),
        quality_scores=np.asarray(final_quality_scores, dtype=np.float32),
        sample_paths=np.asarray(final_sample_paths, dtype=object),
        all_sample_paths=np.asarray(all_sample_paths, dtype=object),
        all_names=np.asarray(all_sample_names, dtype=object),
        all_quality_scores=np.asarray(all_quality_scores, dtype=np.float32),
        pruned_mask=np.asarray(pruned_mask, dtype=np.uint8),
        build_version=np.asarray([BUILD_VERSION], dtype=object),
    )
    print(f"[DONE] DB saved: {out_npz}")
    print(
        f"[STAT] entries={len(final_names)}, total_imgs={total_imgs}, valid_imgs={ok_imgs}, "
        f"aggregate={aggregate}, build_version={BUILD_VERSION}"
    )


def main():
    ap = argparse.ArgumentParser(description="Build face embedding DB from /root/face_db/raw/<name>/*.jpg")
    ap.add_argument("--face-root", default="/root/face_db/raw")
    ap.add_argument("--out", default="/root/face_db/embeddings.npz")
    ap.add_argument("--det-model", default="/root/face_models/face_detection_yunet_2023mar.onnx")
    ap.add_argument("--rec-model", default="/root/face_models/face_recognition_sface_2021dec.onnx")
    ap.add_argument("--det-thres", type=float, default=0.85)
    ap.add_argument("--aggregate", choices=["all", "mean", "all_pruned", "topk", "mean_pruned"], default="all_pruned")
    ap.add_argument("--outlier-drop-ratio", type=float, default=0.15)
    ap.add_argument("--topk", type=int, default=12)
    args = ap.parse_args()
    if not os.path.exists(args.face_root):
        raise FileNotFoundError(f"face root not found: {args.face_root}")

    build_db(
        args.face_root,
        args.out,
        args.det_model,
        args.rec_model,
        args.det_thres,
        args.aggregate,
        outlier_drop_ratio=max(0.0, min(0.45, float(args.outlier_drop_ratio))),
        topk=max(1, int(args.topk)),
    )


if __name__ == "__main__":
    main()
