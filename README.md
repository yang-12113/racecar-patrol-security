# Racecar Patrol Security

Mainline repository for the patrol-security racecar project.

This repository is intended to be the code mainline for day-to-day work:

- `face_tools/`: patrol, follow, alert, face, and helper scripts
- `home/racecar/src/`: ROS2 workspace packages used on the car
- `home/racecar/*.sh`: top-level on-car launch and utility scripts
- `docs/`: handoff notes, logs, and recovery references

Recommended workflow:

1. Edit and test code from this repository.
2. Commit stable milestones to GitHub.
3. Keep models, face databases, bag files, and full-system backups on the USB drive.
4. Sync tested code back to the car explicitly.

This repository intentionally excludes large runtime assets and generated files so Git stays focused on source, configuration, and documentation.
