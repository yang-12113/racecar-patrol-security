import argparse
import json
import os
import shutil
import subprocess
import tempfile
import time
from collections import Counter, deque
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


def l2_normalize(v):
    n = np.linalg.norm(v)
    if n < 1e-12:
        return v
    return v / n


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
    return np.concatenate(mats, axis=0).astype(np.float32, copy=False)


def postprocess(pred, ratio, dwdh, orig_shape, conf_thres=0.25, iou_thres=0.45, target_cls=0):
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
        cls = int(cls_id[i])
        if target_cls >= 0 and cls != target_cls:
            continue
        x1, y1, x2, y2 = xyxy[i].tolist()
        out.append(
            {
                "cls": cls,
                "conf": float(scores[i]),
                "bbox": [int(x1), int(y1), int(x2), int(y2)],
            }
        )
    return out


def iou_xyxy(a, b):
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b
    inter_x1 = max(ax1, bx1)
    inter_y1 = max(ay1, by1)
    inter_x2 = min(ax2, bx2)
    inter_y2 = min(ay2, by2)
    iw = max(0.0, inter_x2 - inter_x1)
    ih = max(0.0, inter_y2 - inter_y1)
    inter = iw * ih
    if inter <= 0:
        return 0.0
    a_area = max(0.0, (ax2 - ax1) * (ay2 - ay1))
    b_area = max(0.0, (bx2 - bx1) * (by2 - by1))
    union = a_area + b_area - inter
    if union <= 0:
        return 0.0
    return inter / union


class Track:
    def __init__(self, track_id, bbox, conf, frame_id):
        self.id = track_id
        self.bbox = bbox
        self.conf = conf
        self.last_seen = frame_id
        self.lost = 0
        self.name_history = deque(maxlen=8)
        self.name = "unknown"
        self.score = 0.0
        self.unknown_streak = 0
        self.last_face_frame = -999999
        self.alarmed_once = False

    def center(self):
        x1, y1, x2, y2 = self.bbox
        return (0.5 * (x1 + x2), 0.5 * (y1 + y2))

    def size(self):
        x1, y1, x2, y2 = self.bbox
        return (max(0, x2 - x1), max(0, y2 - y1))

    def update_name(self, new_name, new_score):
        self.name_history.append(new_name)
        self.score = max(self.score * 0.8, float(new_score))

        cnt = Counter(self.name_history)
        if not cnt:
            self.name = "unknown"
            return
        best_name, best_count = cnt.most_common(1)[0]
        if best_name != "unknown" and best_count >= 2:
            self.name = best_name
            self.unknown_streak = 0
        else:
            self.name = "unknown"


class IOUTracker:
    def __init__(self, iou_thres=0.35, max_lost=20):
        self.iou_thres = iou_thres
        self.max_lost = max_lost
        self.next_id = 1
        self.tracks = {}

    def update(self, detections, frame_id):
        track_ids = list(self.tracks.keys())
        pairs = []
        for ti, tid in enumerate(track_ids):
            tb = self.tracks[tid].bbox
            for di, det in enumerate(detections):
                iou = iou_xyxy(tb, det["bbox"])
                if iou >= self.iou_thres:
                    pairs.append((iou, ti, di))
        pairs.sort(reverse=True, key=lambda x: x[0])

        used_tracks = set()
        used_dets = set()
        for _, ti, di in pairs:
            tid = track_ids[ti]
            if tid in used_tracks or di in used_dets:
                continue
            tr = self.tracks[tid]
            det = detections[di]
            tr.bbox = det["bbox"]
            tr.conf = det["conf"]
            tr.last_seen = frame_id
            tr.lost = 0
            used_tracks.add(tid)
            used_dets.add(di)

        for tid in track_ids:
            if tid not in used_tracks:
                self.tracks[tid].lost += 1

        for di, det in enumerate(detections):
            if di in used_dets:
                continue
            tid = self.next_id
            self.next_id += 1
            self.tracks[tid] = Track(tid, det["bbox"], det["conf"], frame_id)

        dead = [tid for tid, tr in self.tracks.items() if tr.lost > self.max_lost]
        for tid in dead:
            del self.tracks[tid]

        return list(self.tracks.values())


def build_haar_hog():
    cascade = cv2.CascadeClassifier(cv2.data.haarcascades + "haarcascade_frontalface_default.xml")
    hog = cv2.HOGDescriptor(
        _winSize=(64, 64),
        _blockSize=(16, 16),
        _blockStride=(8, 8),
        _cellSize=(8, 8),
        _nbins=9,
    )
    return cascade, hog


def choose_face_backend(det_model, rec_model, det_thres=0.85):
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
    cascade, hog = build_haar_hog()
    return "haar", cascade, hog


def load_face_db(db_path):
    if not os.path.exists(db_path):
        return [], np.zeros((0, 1024), dtype=np.float32), "unknown"
    data = np.load(db_path, allow_pickle=True)
    names = data["names"].tolist()
    feats = data["feats"].astype(np.float32)
    feats = np.asarray([l2_normalize(f) for f in feats], dtype=np.float32)
    method = "unknown"
    if "method" in data:
        method = str(data["method"][0])
    return names, feats, method


def detect_largest_face_sface(detector, img):
    h, w = img.shape[:2]
    if h < 20 or w < 20:
        return None
    detector.setInputSize((w, h))
    _, faces = detector.detect(img)
    if faces is None or len(faces) == 0:
        return None
    areas = faces[:, 2] * faces[:, 3]
    return faces[int(np.argmax(areas))]


def detect_all_faces_sface(detector, img):
    h, w = img.shape[:2]
    if h < 20 or w < 20:
        return []
    detector.setInputSize((w, h))
    _, faces = detector.detect(img)
    if faces is None or len(faces) == 0:
        return []
    return [faces[i] for i in range(len(faces))]


def detect_all_faces_haar(cascade, img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gray = cv2.equalizeHist(gray)
    faces = cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(40, 40))
    return list(faces)


def extract_face_feature_haar(cascade, hog, img):
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


def recognize_face_in_person(face_mode, detector, recognizer, db_names, db_feats, frame, person_bbox, match_thres):
    if len(db_names) == 0 or len(db_feats) == 0:
        return "unknown", 0.0

    x1, y1, x2, y2 = person_bbox
    h, w = frame.shape[:2]
    x1 = max(0, min(w - 1, x1))
    y1 = max(0, min(h - 1, y1))
    x2 = max(0, min(w - 1, x2))
    y2 = max(0, min(h - 1, y2))
    if x2 <= x1 + 20 or y2 <= y1 + 20:
        return "unknown", 0.0

    person = frame[y1:y2, x1:x2]
    ph, pw = person.shape[:2]
    if ph < 80 or pw < 60:
        return "unknown", 0.0

    person_top = person[: int(ph * 0.75), :]
    if face_mode == "sface":
        face = detect_largest_face_sface(detector, person_top)
        if face is None:
            return "unknown", 0.0
        aligned = recognizer.alignCrop(person_top, face)
        feat = recognizer.feature(aligned).flatten().astype(np.float32)
        feat = l2_normalize(feat)
    else:
        feat = extract_face_feature_haar(detector, recognizer, person_top)
        if feat is None:
            return "unknown", 0.0

    scores = db_feats @ feat
    best_idx = int(np.argmax(scores))
    best_score = float(scores[best_idx])
    if best_score >= match_thres:
        return db_names[best_idx], best_score
    return "unknown", best_score


def recognize_faces_in_frame(face_mode, detector, recognizer, db_names, db_feats, frame, match_thres):
    if len(db_names) == 0 or len(db_feats) == 0:
        return []

    out = []
    if face_mode == "sface":
        faces = detect_all_faces_sface(detector, frame)
        for face in faces:
            x, y, w, h = face[:4].astype(np.int32).tolist()
            if w < 20 or h < 20:
                continue
            aligned = recognizer.alignCrop(frame, face)
            feat = recognizer.feature(aligned).flatten().astype(np.float32)
            feat = l2_normalize(feat)
            scores = db_feats @ feat
            best_idx = int(np.argmax(scores))
            best_score = float(scores[best_idx])
            name = db_names[best_idx] if best_score >= match_thres else "unknown"
            out.append(
                {
                    "bbox": [x, y, x + w, y + h],
                    "name": name,
                    "score": best_score,
                }
            )
        return out

    faces = detect_all_faces_haar(detector, frame)
    for (x, y, w, h) in faces:
        roi = frame[y : y + h, x : x + w]
        feat = extract_face_feature_haar(detector, recognizer, roi)
        if feat is None:
            continue
        scores = db_feats @ feat
        best_idx = int(np.argmax(scores))
        best_score = float(scores[best_idx])
        name = db_names[best_idx] if best_score >= match_thres else "unknown"
        out.append(
            {
                "bbox": [int(x), int(y), int(x + w), int(y + h)],
                "name": name,
                "score": best_score,
            }
        )
    return out


def match_face_to_track(face_rows, person_bbox):
    if not face_rows:
        return None
    x1, y1, x2, y2 = person_bbox
    best = None
    best_score = -1.0
    for row in face_rows:
        fx1, fy1, fx2, fy2 = row["bbox"]
        cx = 0.5 * (fx1 + fx2)
        cy = 0.5 * (fy1 + fy2)
        if x1 <= cx <= x2 and y1 <= cy <= y2:
            face_area = max(0, fx2 - fx1) * max(0, fy2 - fy1)
            rank = (float(row["score"]), float(face_area))
            if rank > (best_score, -1):
                best = row
                best_score = float(row["score"])
    return best


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


def trigger_alarm(alarm_cmd):
    try:
        subprocess.Popen(alarm_cmd, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        print("[ALARM] triggered:", alarm_cmd)
    except Exception as e:
        print("[ALARM] failed:", e)


def atomic_write_json(path, data):
    if not path:
        return
    tmp_dir = os.path.dirname(path) or "."
    os.makedirs(tmp_dir, exist_ok=True)
    fd, tmp_path = tempfile.mkstemp(prefix=".face_follow_", suffix=".json", dir=tmp_dir)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as f:
            json.dump(data, f, ensure_ascii=False)
        os.replace(tmp_path, path)
    finally:
        if os.path.exists(tmp_path):
            try:
                os.remove(tmp_path)
            except OSError:
                pass


def choose_follow_target(tracks, follow_policy, follow_name, min_unknown_streak=3, preferred_track_id=None):
    if not tracks:
        return None

    fresh_tracks = [tr for tr in tracks if getattr(tr, "lost", 999999) <= 0]
    if not fresh_tracks:
        return None

    def rank_key(tr):
        x1, y1, x2, y2 = tr.bbox
        area = max(0, x2 - x1) * max(0, y2 - y1)
        return (float(tr.score), float(tr.conf), area)

    if follow_policy == "none":
        return None

    def prefer_existing(candidates):
        if preferred_track_id is None:
            return None
        for tr in candidates:
            if int(tr.id) == int(preferred_track_id):
                return tr
        return None

    if follow_policy == "owner" and follow_name:
        exact = [tr for tr in fresh_tracks if tr.name == follow_name and tr.score >= 0.35]
        if exact:
            keep = prefer_existing(exact)
            if keep is not None:
                return keep
            return max(exact, key=rank_key)
        return None

    if follow_policy == "unknown":
        unknowns = [tr for tr in fresh_tracks if tr.name == "unknown" and tr.unknown_streak >= min_unknown_streak]
        if unknowns:
            keep = prefer_existing(unknowns)
            if keep is not None:
                return keep
            return max(unknowns, key=rank_key)
        return None

    named = [tr for tr in fresh_tracks if tr.name != "unknown" and tr.score >= 0.35]
    if named:
        keep = prefer_existing(named)
        if keep is not None:
            return keep
        return max(named, key=rank_key)
    return None


def write_follow_state(state_file, track, frame_id, frame_shape, follow_name):
    h, w = frame_shape[:2]
    if track is None:
        atomic_write_json(
            state_file,
            {
                "active": False,
                "frame": int(frame_id),
                "ts": time.time(),
                "follow_name": follow_name,
            },
        )
        return

    x1, y1, x2, y2 = track.bbox
    cx = 0.5 * (x1 + x2)
    cy = 0.5 * (y1 + y2)
    area_ratio = max(0.0, (x2 - x1) * (y2 - y1)) / max(1.0, float(w * h))
    atomic_write_json(
        state_file,
        {
            "active": True,
            "frame": int(frame_id),
            "ts": time.time(),
            "follow_name": follow_name,
            "track_id": int(track.id),
            "name": track.name,
            "score": float(track.score),
            "conf": float(track.conf),
            "bbox": [int(x1), int(y1), int(x2), int(y2)],
            "cx": float(cx),
            "cy": float(cy),
            "width": int(w),
            "height": int(h),
            "area_ratio": float(area_ratio),
        },
    )


def main():
    ap = argparse.ArgumentParser(description="YOLO OM + TrackID + Face Recognition + Unknown Alarm")
    ap.add_argument("--model", default="/root/yolov5s_310b.om")
    ap.add_argument("--device", type=int, default=0)
    ap.add_argument("--camera", type=int, default=0)
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--conf", type=float, default=0.35)
    ap.add_argument("--iou", type=float, default=0.45)
    ap.add_argument("--target-cls", type=int, default=0, help="-1 keeps all classes, 0 keeps person")
    ap.add_argument("--infer-interval", type=int, default=2)
    ap.add_argument("--face-interval", type=int, default=6)
    ap.add_argument("--track-iou", type=float, default=0.35)
    ap.add_argument("--track-max-lost", type=int, default=20)
    ap.add_argument("--print-every", type=int, default=10)
    ap.add_argument("--record-step", type=int, default=2)
    ap.add_argument("--det-model", default="/root/face_models/face_detection_yunet_2023mar.onnx")
    ap.add_argument("--rec-model", default="/root/face_models/face_recognition_sface_2021dec.onnx")
    ap.add_argument("--face-db", default="/root/face_db/embeddings.npz")
    ap.add_argument("--match-thres", type=float, default=0.36)
    ap.add_argument("--unknown-frames", type=int, default=12)
    ap.add_argument("--alarm-cooldown", type=float, default=8.0)
    ap.add_argument("--alarm-cmd", default="/usr/local/miniconda3/bin/python /root/alarm_atlas.py 2")
    ap.add_argument("--follow-name", default="owner", help="Preferred recognized name to follow")
    ap.add_argument("--follow-policy", default="owner", choices=["owner", "named", "unknown", "none"])
    ap.add_argument("--follow-unknown-min-frames", type=int, default=6)
    ap.add_argument("--state-file", default="/tmp/face_follow_state.json", help="Shared state file for follow controller")
    ap.add_argument("--no-record", action="store_true", help="Disable temporary video recording")
    ap.add_argument("--no-show", action="store_true")
    args = ap.parse_args()

    args.infer_interval = max(1, args.infer_interval)
    args.face_interval = max(1, args.face_interval)
    args.record_step = max(1, args.record_step)

    backend, sess = load_backend(args.model, args.device)
    print(
        f"backend={backend}, model={args.model}, infer_interval={args.infer_interval}, "
        f"face_interval={args.face_interval}, target_cls={args.target_cls}"
    )

    db_names, db_feats, db_method = load_face_db(args.face_db)
    face_mode, detector, recognizer = choose_face_backend(args.det_model, args.rec_model, det_thres=0.85)
    face_enabled = len(db_names) > 0 and len(db_feats) > 0
    if face_enabled:
        if db_method != "unknown" and db_method != face_mode:
            print(
                f"[FACE] DB method={db_method}, runtime method={face_mode}. "
                "Please rebuild DB with current backend."
            )
            face_enabled = False
        else:
            print(f"[FACE] backend={face_mode}, loaded persons={len(db_names)} from {args.face_db}")
    if not face_enabled:
        print("[FACE] disabled (empty DB or method mismatch).")

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

    tracker = IOUTracker(iou_thres=args.track_iou, max_lost=args.track_max_lost)
    last_alarm_ts = 0.0
    last_dets = []
    last_infer_ms = 0.0
    face_rows = []

    fid = 0
    last_follow_track_id = None
    while True:
        ok, frame = cap.read()
        if not ok:
            break

        if writer is None and not args.no_record:
            h, w = frame.shape[:2]
            writer, tmp_video = make_temp_writer(w, h, fps, args.record_step)
            print(f"[Recorder] 临时录制中: {tmp_video}")

        run_this_frame = (fid == 0) or (fid % args.infer_interval == 0)
        if run_this_frame:
            img, ratio, dwdh = letterbox(frame, (args.imgsz, args.imgsz))
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
            inp = np.transpose(img, (2, 0, 1))[None, ...]
            inp = np.ascontiguousarray(inp)

            t0 = time.time()
            raw = run_infer(backend, sess, inp)
            pred = normalize_output(raw)
            dets = postprocess(pred, ratio, dwdh, frame.shape, args.conf, args.iou, args.target_cls)
            last_infer_ms = (time.time() - t0) * 1000.0
            last_dets = dets
        else:
            dets = last_dets

        tracks = tracker.update(dets, fid)
        now = time.time()

        if face_enabled and (fid == 0 or fid % args.face_interval == 0):
            face_rows = recognize_faces_in_frame(
                face_mode,
                detector,
                recognizer,
                db_names,
                db_feats,
                frame,
                args.match_thres,
            )

        out_rows = []
        for tr in tracks:
            x1, y1, x2, y2 = tr.bbox

            if face_enabled and (fid == 0 or fid % args.face_interval == 0):
                matched_face = match_face_to_track(face_rows, tr.bbox)
                if matched_face is not None:
                    tr.update_name(matched_face["name"], matched_face["score"])
                else:
                    tr.update_name("unknown", 0.0)
                tr.last_face_frame = fid

            if tr.name == "unknown":
                tr.unknown_streak += 1
            else:
                tr.unknown_streak = 0
                tr.alarmed_once = False

            if (
                face_enabled
                and tr.name == "unknown"
                and tr.unknown_streak >= args.unknown_frames
                and (now - last_alarm_ts) >= args.alarm_cooldown
                and not tr.alarmed_once
            ):
                trigger_alarm(args.alarm_cmd)
                tr.alarmed_once = True
                last_alarm_ts = now

            label = f"id{tr.id}:{tr.name}"
            if tr.score > 0:
                label += f" {tr.score:.2f}"

            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, label, (x1, max(0, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            out_rows.append(
                {
                    "id": tr.id,
                    "name": tr.name,
                    "score": round(float(tr.score), 3),
                    "bbox": [int(x1), int(y1), int(x2), int(y2)],
                    "conf": round(float(tr.conf), 3),
                }
            )

        if args.print_every > 0 and fid % args.print_every == 0:
            print(
                json.dumps(
                    {
                        "frame": fid,
                        "infer_ms": round(float(last_infer_ms), 2),
                        "tracks": out_rows,
                    },
                    ensure_ascii=False,
                )
            )

        follow_target = choose_follow_target(
            tracks,
            args.follow_policy,
            args.follow_name,
            min_unknown_streak=max(1, args.follow_unknown_min_frames),
            preferred_track_id=last_follow_track_id,
        )
        last_follow_track_id = int(follow_target.id) if follow_target is not None else None
        write_follow_state(
            args.state_file,
            follow_target,
            fid,
            frame.shape,
            args.follow_name,
        )

        if writer is not None and fid % args.record_step == 0:
            writer.write(frame)

        if not args.no_show:
            cv2.imshow("ascend_face_track_alarm", frame)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")):
                break

        fid += 1

    cap.release()
    if writer is not None:
        writer.release()
    cv2.destroyAllWindows()
    write_follow_state(args.state_file, None, fid, (args.height, args.width, 3), args.follow_name)
    if not args.no_record:
        ask_save_recording(tmp_video)


if __name__ == "__main__":
    main()
