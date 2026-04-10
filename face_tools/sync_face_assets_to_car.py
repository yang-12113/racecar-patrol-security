import argparse
import subprocess
import sys
from pathlib import Path


def run_cmd(cmd, check=True):
    print("[CMD]", " ".join(cmd))
    return subprocess.run(cmd, check=check)


def scp_path(src, dst, recursive=False):
    cmd = [
        "scp",
        "-o",
        "StrictHostKeyChecking=no",
        "-o",
        "UserKnownHostsFile=NUL",
    ]
    if recursive:
        cmd.append("-r")
    cmd.extend([str(src), str(dst)])
    run_cmd(cmd)


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


def resolve_models_dir(local_root):
    candidates = [local_root / "models", local_root / "face_models"]
    for path in candidates:
        if path.exists():
            return path
    return candidates[0]


def resolve_face_root(local_root):
    candidates = [local_root / "captured_faces", local_root / "face_db" / "raw"]
    for path in candidates:
        if path.exists():
            return path
    return candidates[0]


def push_to_car(args):
    local_root = Path(args.local_root)
    models_dir = resolve_models_dir(local_root)
    captured_root = resolve_face_root(local_root)

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
        local_root / "run_mapping_stack.sh",
        local_root / "save_current_map.sh",
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
        scp_path(p, f"{args.user}@{args.host}:{args.remote_tools_dir}/")
    for p in model_files:
        scp_path(p, f"{args.user}@{args.host}:{args.remote_model_dir}/")
    scp_path(face_source, f"{args.user}@{args.host}:{args.remote_face_root}/raw/", recursive=True)

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
                f"chmod +x {args.remote_tools_dir}/run_mapping_stack.sh",
                f"chmod +x {args.remote_tools_dir}/save_current_map.sh",
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
            f"--rec-model {args.remote_model_dir}/face_recognition_sface_2021dec.onnx "
            f"--aggregate {args.rebuild_aggregate} "
            f"--outlier-drop-ratio {args.outlier_drop_ratio} "
            f"--topk {args.topk}"
        )
        ssh_run(args.user, args.host, rebuild_cmd)


def pull_from_car(args):
    local_root = Path(args.local_root)
    local_face_db = local_root / "face_db"
    local_face_raw = local_face_db / "raw"
    local_models = local_root / "face_models"
    local_face_db.mkdir(parents=True, exist_ok=True)
    local_face_raw.parent.mkdir(parents=True, exist_ok=True)
    local_models.mkdir(parents=True, exist_ok=True)

    scp_path(f"{args.user}@{args.host}:{args.remote_face_root}/raw", local_face_raw.parent, recursive=True)
    scp_path(f"{args.user}@{args.host}:{args.remote_face_root}/embeddings.npz", local_face_db / "embeddings.npz")
    scp_path(f"{args.user}@{args.host}:{args.remote_model_dir}", local_root, recursive=True)


def main():
    ap = argparse.ArgumentParser(description="Push/pull face assets and scripts between local machine and the car.")
    ap.add_argument("--mode", choices=["push", "pull"], required=True)
    ap.add_argument("--host", default="192.168.5.100")
    ap.add_argument("--user", default="root")
    ap.add_argument("--local-root", default=str(Path(__file__).resolve().parent))
    ap.add_argument("--remote-tools-dir", default="/root/face_tools")
    ap.add_argument("--remote-model-dir", default="/root/face_models")
    ap.add_argument("--remote-face-root", default="/root/face_db")
    ap.add_argument("--person", default="", help="Optional single person name to upload when mode=push")
    ap.add_argument("--skip-db", action="store_true", help="Skip remote DB rebuild when mode=push")
    ap.add_argument("--rebuild-aggregate", choices=["all", "all_pruned", "topk", "mean_pruned"], default="all_pruned")
    ap.add_argument("--outlier-drop-ratio", type=float, default=0.15)
    ap.add_argument("--topk", type=int, default=12)
    args = ap.parse_args()

    if args.mode == "push":
        push_to_car(args)
    else:
        pull_from_car(args)

    print(f"[DONE] Sync complete. mode={args.mode}")


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print("[ERROR]", e)
        sys.exit(1)
