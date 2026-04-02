# AI Handoff - 2026-04-01

## 使用说明
- 最新交接请优先看这份文件。
- 不删除用户任何文件。
- 不做全盘扫描，优先只看关键目录。
- 当前小车网络断开时，优先做本地代码整理、日志更新和离线修复。

## 项目目标
- 设备: Atlas200 DK / Ascend310B / ARM aarch64
- 目标链路: `ONNX -> OM -> 实时检测 -> 人脸识别 -> 巡航 -> 避障 -> 陌生人追踪 -> 返回原路线`
- 当前阶段重点:
  1. 让 `owner` / `unknown` 识别与跟随稳定可控
  2. 保证目标离开后小车能及时回中位停止
  3. 在此基础上再做巡航状态机

## 小车配置

### 1. 硬件
- 主控: Atlas 200 DK (`c78`)
- NPU: Ascend310B
- 架构: ARM `aarch64`
- 内存: 约 `1.8GB`
- 激光雷达: `RPLIDAR X4`
- 文档记录相机: `RealSense D435i`
- 当前运行假设:
  - 项目必须先按 `RGB + 激光雷达` 跑通
  - 深度能力只作为后续优化项
  - 因为用户口头反馈“当前相机不一定在用深度”

### 2. 网络
- 小车 IP: `192.168.5.100`
- 当前常用连接方式:
  - 电脑通过手机 `USB` 共享网络上网
  - 小车通过局域网或直连方式 `ssh root@192.168.5.100`
- 如果网络不稳:
  - 以 `F:\\小车文件\\face_tools` 为唯一可信源码
  - 小车重连后先重新同步, 再做实机测试

### 3. 推理与模型
- CANN / ACL 已安装并验证通过
- 目标检测模型:
  - ONNX: `/root/yolo/yolov5s.onnx`
  - OM: `/root/yolov5s_310b.om`
- 人脸模型:
  - `/root/face_models/face_detection_yunet_2023mar.onnx`
  - `/root/face_models/face_recognition_sface_2021dec.onnx`
- 人脸库:
  - 原始样本: `/root/face_db/raw`
  - Embeddings: `/root/face_db/embeddings.npz`

### 4. ROS2 与车控
- ROS2 版本: `Humble`
- 关键控制话题: `/car_cmd_vel`
- 当前控制语义:
  - 中位速度: `linear.x = 1500`
  - 中位转向: `angular.z = 90`
  - 转向限幅: `45 ~ 135`
- 关键参考文件:
  - `/home/racecar/nav.sh`
  - `/home/racecar/src/racecar/launch/classroom_nav.launch.py`
  - `/home/racecar/src/racecar/config/nav/navigation.yaml`
  - `/home/racecar/src/racecar/src/car_controller_new.cpp`

## 我们的目标

### 1. 最终比赛目标
- 小车能在规定地图中自主巡航
- 巡航过程中能识别“已录入人脸”和“陌生人”
- 发现陌生人后中断巡航并进行跟踪
- 目标离开警戒范围后返回原巡航路线继续巡航
- 全程具备基本避障能力

### 2. 当前不优先但后续要做
- 多个人脸同时出现时的冲突处理
- 更稳定的目标优先级策略
- 将“直接发 `/car_cmd_vel` 跟随”升级为更稳的导航式跟随

### 3. 当前工程推进顺序
1. 先收口 `owner` / `unknown` 的识别与跟随稳定性
2. 再实现 `PATROL -> ALERT_TRACK -> RETURN_TO_ROUTE -> SAFE_STOP` 状态机
3. 再把跟随阶段和避障能力整合到同一套控制逻辑

## 已完成状态

### 1. CANN 与模型转换
- 小车端已完成 CANN 安装与验证。
- `atc`、`acl`、`npu-smi info` 均验证通过。
- YOLO ONNX 已转成 OM:
  - ONNX: `/root/yolo/yolov5s.onnx`
  - OM: `/root/yolov5s_310b.om`
  - 关键参数: `--framework=5 --input_shape=images:1,3,640,640 --soc_version=Ascend310B1`

### 2. 基础检测脚本
- 小车端已有实时检测脚本:
  - `/root/yolo_om_cam_bbox.py`
  - `/root/run_yolo_bbox.sh`
- 支持:
  - 摄像头实时检测
  - 退出询问是否保存录像
  - 自定义保存目录与文件名
- 已做过轻量提速:
  - `--infer-interval`
  - `--print-every`
  - `--record-step`
  - `--target-cls`

### 3. 本地人脸识别工具链
- 本地工作目录: `F:\小车文件\face_tools`
- 已有文件:
  - `build_face_db.py`
  - `enroll_face_and_sync.py`
  - `sync_face_assets_to_car.py`
  - `capture_face_on_car.py`
  - `yolo_face_track_alarm.py`
  - `face_follow_controller.py`
  - `run_yolo_face_track.sh`
  - `run_face_follow_stack.sh`
  - `run_follow_demo_full_stack.sh`
  - `run_intruder_follow_demo_full_stack.sh`
- 本地 ONNX 人脸模型已准备并验证:
  - `F:\小车文件\face_tools\models\face_detection_yunet_2023mar.onnx`
  - `F:\小车文件\face_tools\models\face_recognition_sface_2021dec.onnx`
- 本地 `owner` 样本已录入并建库成功:
  - 样本目录: `F:\小车文件\face_tools\captured_faces\owner`
  - 本地库: `F:\小车文件\face_tools\captured_faces\local_embeddings.npz`

### 4. 跟随链路设计
- 当前链路为:
  - `yolo_face_track_alarm.py` 写 `/tmp/face_follow_state.json`
  - `face_follow_controller.py` 读取状态并发布 `/car_cmd_vel`
- 这样做的原因:
  - 避免 ACL/CANN 与 ROS2 强耦合到一个 Python 环境
  - 便于单独调试识别与底盘控制

## 已确认的关键事实

### 1. RViz 自动弹出原因
- 根因不是用户误操作。
- 来自 `/home/racecar/src/racecar/launch/Run_car.launch.py` 包含的激光雷达启动链。
- `lslidar_launch.py` 默认会拉起 `rviz2`。
- 本地完整启动脚本已经加了 `pkill -f rviz2`。

### 2. 视频保存询问消失的原因
- 之前为了提速，`run_face_follow_stack.sh` 曾强制带 `--no-record`。
- 本地版本已移除该强制项，退出时应恢复询问保存录像。

### 3. “识别到了但不跟随”的早期根因
- 一部分是控制器最早只认 `owner`，导致 `unknown` 模式不动。
- 本地版本已加 `--target-mode owner|unknown|any`。
- 另一部分是默认速度参数过于保守，本地版本已提高默认响应。

### 4. “人物离开了车还继续跑”的高概率根因
- 早期目标选择会沿用非新鲜轨迹，目标消失后仍可能沿旧轨迹继续给出活动目标。
- 早期 shell 包装脚本使用 `exec`，导致退出时 cleanup 不执行，控制器和最后一条速度命令可能残留。
- 真实小车上表现为:
  - 人走开后车继续前进
  - 关 ESC 暂停后，重新上电又继续
- 本地已做的修复:
  - `choose_follow_target()` 只接受 `lost <= 0` 的新鲜轨迹
  - `run_face_follow_stack.sh` 不再 `exec` 主脚本，而是后台启动并 `wait`
  - cleanup 时会主动发中位停止命令
  - cleanup 时会清空状态文件
  - 启动前先杀掉残留的 `face_follow_controller.py` / `yolo_face_track_alarm.py`

## 2026-04-01 本地新增修复

### 1. `yolo_face_track_alarm.py`
- 人脸识别改成“全帧先找脸，再关联到人框”。
- 已支持:
  - `--follow-policy owner|named|unknown|none`
  - `--follow-unknown-min-frames`
- 目标选择逻辑已改为只选 `fresh_tracks`。
- 退出前会写入 `active=false` 的状态。

### 2. `face_follow_controller.py`
- 已支持 `--target-mode owner|unknown|any`。
- 默认参数较之前更积极，便于真正驱动车体移动。
- 本地新增关闭保护:
  - `publish_neutral_burst()` 连续多次发布中位指令
  - 退出时对 ROS 上下文失效做了容错，不再依赖单次 `publish_neutral()`

### 3. `run_face_follow_stack.sh`
- 启动前先清理残留跟随进程。
- 启动前先发一次中位停止命令。
- cleanup 时:
  - 杀掉跟随与识别进程
  - 清空状态文件
  - 再次发布中位停止命令
- 不再强制 `--no-record`。

### 4. `run_follow_demo_full_stack.sh` / `run_intruder_follow_demo_full_stack.sh`
- 已加入自动关闭 RViz 的逻辑。
- 仍保留 `Run_car.launch.py` 作为底盘驱动启动入口。

## 当前阻塞
- 当前轮次里，小车网络不可达，无法确认远端是否已经同步到“4 月 1 日本地最新版”。
- 所以正确策略是:
  1. 把本地版本视为真源
  2. 小车重连后先重新同步
  3. 再做实车验证

## 小车相关关键路径

### 小车端
- `/root/yolov5s_310b.om`
- `/root/yolo_om_cam_bbox.py`
- `/root/run_yolo_bbox.sh`
- `/root/face_tools/yolo_face_track_alarm.py`
- `/root/face_tools/face_follow_controller.py`
- `/root/face_tools/run_face_follow_stack.sh`
- `/root/face_tools/run_follow_demo_full_stack.sh`
- `/root/face_tools/run_intruder_follow_demo_full_stack.sh`
- `/root/face_tools/capture_face_on_car.py`
- `/root/alarm_atlas.py`
- `/root/combo_alarm.py`
- `/home/racecar/nav.sh`
- `/home/racecar/src/racecar/launch/classroom_nav.launch.py`
- `/home/racecar/src/racecar/config/nav/navigation.yaml`
- `/home/racecar/src/racecar/src/car_controller_new.cpp`

### 本地端
- `F:\小车文件\face_tools\yolo_face_track_alarm.py`
- `F:\小车文件\face_tools\face_follow_controller.py`
- `F:\小车文件\face_tools\run_face_follow_stack.sh`
- `F:\小车文件\face_tools\run_follow_demo_full_stack.sh`
- `F:\小车文件\face_tools\run_intruder_follow_demo_full_stack.sh`
- `F:\小车文件\face_tools\sync_face_assets_to_car.py`
- `F:\小车文件\AI_HANDOFF_20260331.md`
- `F:\小车文件\AI_HANDOFF_20260401.md`

## 重连后第一优先级动作

### 1. 先重同步本地最新版到小车
```powershell
python "F:\小车文件\face_tools\sync_face_assets_to_car.py" --person owner
```

### 2. 清理小车上可能残留的跟随进程
```bash
pkill -f /root/face_tools/face_follow_controller.py || true
pkill -f /root/face_tools/yolo_face_track_alarm.py || true
printf '{}' > /tmp/face_follow_state.json
```

### 3. 先做“只识别不跟随”验证
```bash
/root/run_yolo_face_track.sh --follow-policy none --infer-interval 2 --face-interval 4 --print-every 10
```

### 4. 再做 `owner` 跟随验证
```bash
/root/face_tools/run_follow_demo_full_stack.sh --infer-interval 2 --face-interval 4 --print-every 10
```
重点观察:
- 人物离开画面后 1 秒内是否回中位
- 退出后是否询问保存录像
- 是否还会再弹 RViz

### 5. 再做 `unknown` 跟随验证
```bash
/root/face_tools/run_intruder_follow_demo_full_stack.sh --infer-interval 2 --face-interval 4 --print-every 10
```

## 比赛版还没做完的工作

### A. 实车跟随可靠性收口
- 实测确认“目标离开后停车”是否已经真正修好。
- 对 `owner` 和 `unknown` 两条链分别实测。
- 必要时继续调:
  - `timeout`
  - `track-max-lost`
  - `follow-unknown-min-frames`
  - `desired-area`
  - `speed-gain`
  - `steer-gain`

### B. 巡航主管节点 `patrol_supervisor.py`
- 目前还没有正式实现。
- 需要负责状态机:
  - `PATROL`
  - `ALERT_TRACK`
  - `RETURN_TO_ROUTE`
  - `SAFE_STOP`
- 推荐实现:
  1. 使用 Nav2 巡航航点
  2. 由视觉监控节点上报陌生人事件
  3. 中断巡航并进入跟随
  4. 目标离开范围后返回最近巡航点继续任务

### C. 跟随阶段的避障策略
- 巡航阶段已有 Nav2 + lidar costmap 基础，来自:
  - `/home/racecar/nav.sh`
  - `/home/racecar/src/racecar/config/nav/navigation.yaml`
- 但当前跟随演示是直接发 `/car_cmd_vel`，还没有把避障也纳入同一个控制层。
- 比赛版建议:
  - 先让跟随只在低速下运行
  - 最终把“跟随目标”转为短目标点，交给 Nav2 局部规划，而不是长期裸发 `/car_cmd_vel`

### D. 多人脸冲突策略
- 用户明确说“现在先不做”，但比赛版迟早要补。
- 推荐以后增加:
  - 目标优先级
  - 目标锁定时长
  - 距离/画面中心联合排序

## 安全注意
- 实车跟随测试前，优先让车轮悬空或留足空场。
- 不要和其他会发布 `/car_cmd_vel` 的节点同时运行。
- 当前项目里容易冲突的节点/脚本包括巡线、检测、导航控制等历史脚本。

## 给下一位 AI 的一句话
- 先把小车重连，再以 `F:\小车文件\face_tools` 为唯一可信源码重新同步。
- 第一验证目标不是“继续加功能”，而是先确认 4 月 1 日本地修复是否已经解决“目标离开后仍持续前进”的问题。
