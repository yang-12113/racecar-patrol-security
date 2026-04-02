import argparse
import subprocess
import sys
from pathlib import Path


def run_cmd(cmd, check=True):
    print("[CMD]", " ".join(cmd))
    return subprocess.run(cmd, check=check)


def scp_to_car(local_path, remote_path):
    run_cmd(
        [
            "scp",
            "-o",
            "StrictHostKeyChecking=no",
            "-o",
            "UserKnownHostsFile=NUL",
            "-r",
            str(local_path),
            remote_path,
        ]
    )


def ssh_run(user, host, remote_cmd):
    run_cmd(
        [
            "ssh",
            "-o",
            "StrictHostKeyChecking=no",
            "-o",
            "UserKnownHostsFile=NUL",
            f"{user}@{host}",
            remote_cmd,
        ]
    )


def main():
    ap = argparse.ArgumentParser(description="Upload face models, scripts and captured faces to the car.")
    ap.add_argument("--host", default="192.168.5.100")
    ap.add_argument("--user", default="root")
    ap.add_argument("--local-root", default=str(Path(__file__).resolve().parent))
    ap.add_argument("--remote-tools-dir", default="/root/face_tools")
    ap.add_argument("--remote-model-dir", default="/root/face_models")
    ap.add_argument("--remote-face-root", default="/root/face_db")
    ap.add_argument("--person", default="", help="Optional single person name under captured_faces to upload")
    ap.add_argument("--skip-db", action="store_true", help="Skip remote DB rebuild")
    args = ap.parse_args()

    local_root = Path(args.local_root)
    models_dir = local_root / "models"
    captured_root = local_root / "captured_faces"

    tool_files = [
        local_root / "build_face_db.py",
        local_root / "yolo_face_track_alarm.py",
        local_root / "run_yolo_face_track.sh",
        local_root / "face_follow_controller.py",
        local_root / "run_face_follow_stack.sh",
        local_root / "run_follow_demo_full_stack.sh",
        local_root / "run_intruder_follow_demo_full_stack.sh",
        local_root / "run_intruder_patrol_demo_full_stack.sh",
        local_root / "run_nav_stack_with_params.sh",
        local_root / "run_navigation_test_full_stack.sh",
        local_root / "capture_face_on_car.py",
        local_root / "capture_patrol_waypoint.py",
        local_root / "patrol_preflight_check.py",
        local_root / "publish_initial_pose.py",
        local_root / "wait_localization_ready.py",
        local_root / "patrol_supervisor.py",
        local_root / "patrol_waypoints_demo.json",
    ]
    model_files = [
        models_dir / "face_detection_yunet_2023mar.onnx",
        models_dir / "face_recognition_sface_2021dec.onnx",
    ]

    missing = [str(p) for p in tool_files + model_files if not p.exists()]
    if missing:
        raise FileNotFoundError("Missing local files: " + ", ".join(missing))

    if args.person:
        face_source = captured_root / args.person
    else:
        face_source = captured_root
    if not face_source.exists():
        raise FileNotFoundError(f"Local face source not found: {face_source}")

    ssh_run(
        args.user,
        args.host,
        f"mkdir -p {args.remote_tools_dir} {args.remote_model_dir} {args.remote_face_root}/raw",
    )

    for p in tool_files:
        scp_to_car(p, f"{args.user}@{args.host}:{args.remote_tools_dir}/")
    for p in model_files:
        scp_to_car(p, f"{args.user}@{args.host}:{args.remote_model_dir}/")
    scp_to_car(face_source, f"{args.user}@{args.host}:{args.remote_face_root}/raw/")

    ssh_run(
        args.user,
        args.host,
        " && ".join(
            [
                f"chmod +x {args.remote_tools_dir}/run_yolo_face_track.sh",
                f"chmod +x {args.remote_tools_dir}/run_face_follow_stack.sh",
                f"chmod +x {args.remote_tools_dir}/run_follow_demo_full_stack.sh",
                f"chmod +x {args.remote_tools_dir}/run_intruder_follow_demo_full_stack.sh",
                f"chmod +x {args.remote_tools_dir}/run_intruder_patrol_demo_full_stack.sh",
                f"chmod +x {args.remote_tools_dir}/run_nav_stack_with_params.sh",
                f"chmod +x {args.remote_tools_dir}/run_navigation_test_full_stack.sh",
                f"/usr/local/miniconda3/bin/python -m py_compile {args.remote_tools_dir}/build_face_db.py",
                f"/usr/local/miniconda3/bin/python -m py_compile {args.remote_tools_dir}/yolo_face_track_alarm.py",
                f"/usr/local/miniconda3/bin/python -m py_compile {args.remote_tools_dir}/capture_face_on_car.py",
                f"/usr/bin/python3 -m py_compile {args.remote_tools_dir}/capture_patrol_waypoint.py",
                f"/usr/bin/python3 -m py_compile {args.remote_tools_dir}/patrol_preflight_check.py",
                f"/usr/bin/python3 -m py_compile {args.remote_tools_dir}/publish_initial_pose.py",
                f"/usr/bin/python3 -m py_compile {args.remote_tools_dir}/wait_localization_ready.py",
                f"/usr/bin/python3 -m py_compile {args.remote_tools_dir}/face_follow_controller.py",
                f"/usr/bin/python3 -m py_compile {args.remote_tools_dir}/patrol_supervisor.py",
            ]
        ),
    )

    if not args.skip_db:
        rebuild_cmd = (
            "source /usr/local/Ascend/ascend-toolkit/set_env.sh; "
            "source /usr/local/Ascend/nnae/set_env.sh; "
            f"/usr/local/miniconda3/bin/python {args.remote_tools_dir}/build_face_db.py "
            f"--face-root {args.remote_face_root}/raw "
            f"--out {args.remote_face_root}/embeddings.npz "
            f"--det-model {args.remote_model_dir}/face_detection_yunet_2023mar.onnx "
            f"--rec-model {args.remote_model_dir}/face_recognition_sface_2021dec.onnx"
        )
        ssh_run(args.user, args.host, rebuild_cmd)

    print("[DONE] Sync complete.")


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print("[ERROR]", e)
        sys.exit(1)
