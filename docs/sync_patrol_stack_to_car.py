import argparse
import subprocess
import sys
from pathlib import Path


def run_cmd(cmd, check=True):
    print("[CMD]", " ".join(str(x) for x in cmd))
    return subprocess.run([str(x) for x in cmd], check=check)


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


def scp_to_dir(user, host, paths, remote_dir):
    run_cmd(
        [
            "scp",
            "-o",
            "StrictHostKeyChecking=no",
            "-o",
            "UserKnownHostsFile=NUL",
            *[str(p) for p in paths],
            f"{user}@{host}:{remote_dir}",
        ]
    )


def main():
    ap = argparse.ArgumentParser(
        description="Sync patrol/follow scripts plus navigation_test assets to the car and optionally rebuild racecar."
    )
    ap.add_argument("--host", default="192.168.5.100")
    ap.add_argument("--user", default="root")
    ap.add_argument("--local-root", default=str(Path(__file__).resolve().parent))
    ap.add_argument("--skip-build", action="store_true")
    args = ap.parse_args()

    local_root = Path(args.local_root)
    face_root = local_root / "face_tools"
    racecar_root = local_root / "home" / "racecar" / "src" / "racecar"

    face_files = [
        face_root / "yolo_face_track_alarm.py",
        face_root / "face_follow_controller.py",
        face_root / "patrol_preflight_check.py",
        face_root / "patrol_supervisor.py",
        face_root / "publish_initial_pose.py",
        face_root / "wait_localization_ready.py",
        face_root / "capture_patrol_waypoint.py",
        face_root / "run_intruder_patrol_demo_full_stack.sh",
        face_root / "run_nav_stack_with_params.sh",
        face_root / "run_navigation_test_full_stack.sh",
        face_root / "patrol_waypoints_demo.json",
    ]
    racecar_files = [
        racecar_root / "scripts" / "navigation_test.py",
        racecar_root / "CMakeLists.txt",
        racecar_root / "package.xml",
    ]
    nav_config_files = [
        racecar_root / "config" / "nav" / "navigation_patrol_safe.yaml",
    ]

    missing = [str(p) for p in face_files + racecar_files + nav_config_files if not p.exists()]
    if missing:
        raise FileNotFoundError("Missing local files: " + ", ".join(missing))

    ssh_run(
        args.user,
        args.host,
        "mkdir -p /root/face_tools /home/racecar/src/racecar/scripts /home/racecar/src/racecar/config/nav",
    )
    scp_to_dir(args.user, args.host, face_files, "/root/face_tools/")
    scp_to_dir(args.user, args.host, [racecar_root / "scripts" / "navigation_test.py"], "/home/racecar/src/racecar/scripts/")
    scp_to_dir(
        args.user,
        args.host,
        [racecar_root / "CMakeLists.txt", racecar_root / "package.xml"],
        "/home/racecar/src/racecar/",
    )
    scp_to_dir(args.user, args.host, nav_config_files, "/home/racecar/src/racecar/config/nav/")
    ssh_run(
        args.user,
        args.host,
        " && ".join(
            [
                "chmod +x /root/face_tools/run_intruder_patrol_demo_full_stack.sh",
                "chmod +x /root/face_tools/run_nav_stack_with_params.sh",
                "chmod +x /root/face_tools/run_navigation_test_full_stack.sh",
                "chmod +x /home/racecar/src/racecar/scripts/navigation_test.py",
                "/usr/local/miniconda3/bin/python -m py_compile /root/face_tools/yolo_face_track_alarm.py",
                "/usr/bin/python3 -m py_compile /root/face_tools/face_follow_controller.py",
                "/usr/bin/python3 -m py_compile /root/face_tools/patrol_preflight_check.py",
                "/usr/bin/python3 -m py_compile /root/face_tools/patrol_supervisor.py",
                "/usr/bin/python3 -m py_compile /root/face_tools/publish_initial_pose.py",
                "/usr/bin/python3 -m py_compile /root/face_tools/wait_localization_ready.py",
                "/usr/bin/python3 -m py_compile /root/face_tools/capture_patrol_waypoint.py",
                "/usr/bin/python3 -m py_compile /home/racecar/src/racecar/scripts/navigation_test.py",
                "bash -n /root/face_tools/run_intruder_patrol_demo_full_stack.sh",
            ]
        ),
    )

    if not args.skip_build:
        ssh_run(
            args.user,
            args.host,
            "bash -lc 'source /opt/ros/humble/setup.bash && cd /home/racecar && colcon build --packages-select racecar'",
        )
        ssh_run(
            args.user,
            args.host,
            "bash -lc 'source /opt/ros/humble/setup.bash && "
            "[ -f /home/racecar/install/setup.bash ] && source /home/racecar/install/setup.bash; "
            "ros2 pkg executables racecar | grep navigation_test'",
        )

    print("[DONE] Patrol stack sync complete.")


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print("[ERROR]", exc)
        sys.exit(1)
