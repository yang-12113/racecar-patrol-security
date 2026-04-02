import argparse
import os
import shutil
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import cv2


def run_cmd(cmd, check=True):
    print("[CMD]", " ".join(cmd))
    return subprocess.run(cmd, check=check)


def open_camera(cam_id):
    cap = cv2.VideoCapture(cam_id, cv2.CAP_DSHOW)
    if not cap.isOpened():
        cap.release()
        cap = cv2.VideoCapture(cam_id)
    return cap


def save_image_unicode(path, frame):
    suffix = Path(path).suffix or ".jpg"
    ok, buf = cv2.imencode(suffix, frame)
    if not ok:
        return False
    buf.tofile(str(path))
    return True


def ascii_model_path(path):
    path = Path(path)
    if all(ord(ch) < 128 for ch in str(path)):
        return str(path)

    cache_dir = Path(tempfile.gettempdir()) / "codex_face_models"
    cache_dir.mkdir(parents=True, exist_ok=True)
    dst = cache_dir / path.name
    if (not dst.exists()) or dst.stat().st_size != path.stat().st_size:
        shutil.copyfile(path, dst)
    return str(dst)


def make_local_face_detector(models_dir, det_thres=0.85):
    models_dir = Path(models_dir)
    det_model = models_dir / "face_detection_yunet_2023mar.onnx"
    if (
        det_model.exists()
        and det_model.stat().st_size > 0
        and hasattr(cv2, "FaceDetectorYN_create")
    ):
        detector = cv2.FaceDetectorYN_create(ascii_model_path(det_model), "", (320, 320), det_thres, 0.3, 5000)
        return "yunet", detector

    cascade = cv2.CascadeClassifier(cv2.data.haarcascades + "haarcascade_frontalface_default.xml")
    return "haar", cascade


def has_face(frame, mode, detector):
    if mode == "yunet":
        h, w = frame.shape[:2]
        detector.setInputSize((w, h))
        _, faces = detector.detect(frame)
        if faces is None or len(faces) == 0:
            return False, []
        boxes = []
        for row in faces:
            x, y, fw, fh = row[:4].astype(int).tolist()
            boxes.append((x, y, fw, fh))
        return True, boxes

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.equalizeHist(gray)
    faces = detector.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(50, 50))
    return len(faces) > 0, faces


def capture_faces(name, out_root, num, cam_id, warmup_sec, every_n_frames, models_dir):
    person_dir = out_root / name
    person_dir.mkdir(parents=True, exist_ok=True)
    det_mode, detector = make_local_face_detector(models_dir)
    print(f"[INFO] Local face detector: {det_mode}")

    cap = open_camera(cam_id)
    if not cap.isOpened():
        raise RuntimeError("Cannot open local front camera.")

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 30)

    print(f"[INFO] Camera opened. Start capture in {warmup_sec}s...")
    t_end = time.time() + warmup_sec
    while time.time() < t_end:
        ok, frame = cap.read()
        if not ok:
            continue
        cv2.putText(frame, "Get ready...", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
        cv2.imshow("face_enroll_local", frame)
        if cv2.waitKey(1) & 0xFF in (27, ord("q")):
            cap.release()
            cv2.destroyAllWindows()
            raise RuntimeError("Canceled by user before capture.")

    saved = 0
    frame_id = 0
    print("[INFO] Capturing...")
    while saved < num:
        ok, frame = cap.read()
        if not ok:
            continue

        found, faces = has_face(frame, det_mode, detector)

        if frame_id % every_n_frames == 0 and found:
            ts = int(time.time() * 1000)
            out = person_dir / f"{name}_{saved:03d}_{ts}.jpg"
            if save_image_unicode(out, frame):
                saved += 1

        vis = frame.copy()
        for (x, y, w, h) in faces[:1]:
            cv2.rectangle(vis, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.putText(
            vis,
            f"{name}: {saved}/{num}  (press q to stop)",
            (16, 32),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0),
            2,
        )
        if not found:
            cv2.putText(
                vis,
                "No face detected, please face the camera",
                (16, 64),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 255),
                2,
            )
        cv2.imshow("face_enroll_local", vis)
        if cv2.waitKey(1) & 0xFF in (27, ord("q")):
            break
        frame_id += 1

    cap.release()
    cv2.destroyAllWindows()
    return person_dir, saved


def sync_to_car(person_dir, user, host, remote_face_root):
    remote_raw_root = f"{remote_face_root}/raw"
    run_cmd(
        [
            "ssh",
            "-o",
            "StrictHostKeyChecking=no",
            "-o",
            "UserKnownHostsFile=NUL",
            f"{user}@{host}",
            f"mkdir -p {remote_raw_root}",
        ]
    )
    run_cmd(
        [
            "scp",
            "-o",
            "StrictHostKeyChecking=no",
            "-o",
            "UserKnownHostsFile=NUL",
            "-r",
            str(person_dir),
            f"{user}@{host}:{remote_raw_root}/",
        ]
    )


def upload_face_models(user, host, local_models_dir, remote_model_dir):
    local_models_dir = Path(local_models_dir)
    det_model = local_models_dir / "face_detection_yunet_2023mar.onnx"
    rec_model = local_models_dir / "face_recognition_sface_2021dec.onnx"
    missing = [str(p) for p in (det_model, rec_model) if not p.exists() or p.stat().st_size <= 0]
    if missing:
        raise FileNotFoundError("Missing local ONNX models: " + ", ".join(missing))

    run_cmd(
        [
            "ssh",
            "-o",
            "StrictHostKeyChecking=no",
            "-o",
            "UserKnownHostsFile=NUL",
            f"{user}@{host}",
            f"mkdir -p {remote_model_dir}",
        ]
    )
    for model_path in (det_model, rec_model):
        run_cmd(
            [
                "scp",
                "-o",
                "StrictHostKeyChecking=no",
                "-o",
                "UserKnownHostsFile=NUL",
                str(model_path),
                f"{user}@{host}:{remote_model_dir}/",
            ]
        )


def rebuild_db(user, host, remote_face_root, det_model, rec_model):
    py = "/usr/local/miniconda3/bin/python"
    build_py = "/root/face_tools/build_face_db.py"
    cmd = (
        "source /usr/local/Ascend/ascend-toolkit/set_env.sh; "
        "source /usr/local/Ascend/nnae/set_env.sh; "
        f"{py} {build_py} "
        f"--face-root {remote_face_root}/raw "
        f"--out {remote_face_root}/embeddings.npz "
        f"--det-model {det_model} "
        f"--rec-model {rec_model}"
    )
    run_cmd(
        [
            "ssh",
            "-o",
            "StrictHostKeyChecking=no",
            "-o",
            "UserKnownHostsFile=NUL",
            f"{user}@{host}",
            cmd,
        ]
    )


def main():
    ap = argparse.ArgumentParser(description="Capture local front-camera face images and sync to car.")
    ap.add_argument("--name", required=True, help="Person name")
    ap.add_argument("--num", type=int, default=20, help="Number of images")
    ap.add_argument("--camera", type=int, default=0, help="Local webcam index")
    ap.add_argument("--warmup", type=int, default=3, help="Countdown seconds")
    ap.add_argument("--every", type=int, default=6, help="Capture one image every N frames")
    ap.add_argument("--host", default="192.168.5.100")
    ap.add_argument("--user", default="root")
    ap.add_argument("--remote-face-root", default="/root/face_db")
    ap.add_argument("--remote-model-dir", default="/root/face_models")
    ap.add_argument(
        "--local-root",
        default=str(Path(__file__).resolve().parent / "captured_faces"),
        help="Local temporary capture folder",
    )
    ap.add_argument(
        "--models-dir",
        default=str(Path(__file__).resolve().parent / "models"),
        help="Local ONNX face model directory",
    )
    ap.add_argument("--keep-local", action="store_true", help="Keep local captured images")
    ap.add_argument("--skip-sync", action="store_true", help="Only capture locally, do not upload now")
    args = ap.parse_args()

    local_root = Path(args.local_root)
    local_root.mkdir(parents=True, exist_ok=True)

    person_dir, saved = capture_faces(
        name=args.name,
        out_root=local_root,
        num=max(1, args.num),
        cam_id=args.camera,
        warmup_sec=max(0, args.warmup),
        every_n_frames=max(1, args.every),
        models_dir=args.models_dir,
    )
    if saved <= 0:
        raise RuntimeError("No face images captured.")

    print(f"[INFO] Captured {saved} images at: {person_dir}")

    if not args.skip_sync:
        try:
            upload_face_models(args.user, args.host, args.models_dir, args.remote_model_dir)
            sync_to_car(person_dir, args.user, args.host, args.remote_face_root)
            rebuild_db(
                args.user,
                args.host,
                args.remote_face_root,
                det_model=f"{args.remote_model_dir}/face_detection_yunet_2023mar.onnx",
                rec_model=f"{args.remote_model_dir}/face_recognition_sface_2021dec.onnx",
            )
            print("[INFO] Face DB rebuilt on car.")
        except Exception as e:
            print("[WARN] Capture finished, but sync/rebuild failed:", e)
            print("[WARN] Local images are kept. Retry later after SSH recovers.")
            args.keep_local = True

    if not args.keep_local:
        shutil.rmtree(person_dir, ignore_errors=True)
        print("[INFO] Local captured images removed (car copy kept).")


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print("[ERROR]", e)
        sys.exit(1)
