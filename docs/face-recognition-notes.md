# AI Handoff - 2026-03-31

> 最新交接请优先查看 `F:\小车文件\AI_HANDOFF_20260401.md`

## 目标
- 设备: Atlas200 DK / Ascend310B / ARM aarch64
- 任务: `ONNX -> OM -> 实时检测 -> 人脸识别 -> 巡航 -> 避障 -> 陌生人追踪 -> 返回原路线`
- 约束: 不删除用户任何文件, 不做全盘扫描, 优先只看关键目录

## 已完成

### 1. CANN 安装与验证
- 已在小车上完成 CANN 安装并验证通过
- 核心结果:
  - `which atc` 有效
  - `python3 -c "import acl"` 成功
  - `npu-smi info` 正常
  - `atc --mode=6 --om=...` 正常

### 2. YOLO ONNX -> OM
- ONNX: `/root/yolo/yolov5s.onnx`
- OM: `/root/yolov5s_310b.om`
- 成功命令核心参数:
  - `--framework=5`
  - `--input_shape=images:1,3,640,640`
  - `--soc_version=Ascend310B1`

### 3. 基础实时检测脚本
- 小车端脚本: `/root/yolo_om_cam_bbox.py`
- 小车端启动器: `/root/run_yolo_bbox.sh`
- 功能:
  - 摄像头实时推理
  - 退出时询问是否保存录像
  - 支持自定义保存目录与文件名
- 已做过提速项:
  - `--infer-interval`
  - `--print-every`
  - `--record-step`
  - `--target-cls`

### 4. 人脸识别链路本地准备完成
- 本地目录: `F:\小车文件\face_tools`
- 已创建文件:
  - `F:\小车文件\face_tools\build_face_db.py`
  - `F:\小车文件\face_tools\enroll_face_and_sync.py`
  - `F:\小车文件\face_tools\sync_face_assets_to_car.py`
  - `F:\小车文件\face_tools\yolo_face_track_alarm.py`
  - `F:\小车文件\face_tools\face_follow_controller.py`
  - `F:\小车文件\face_tools\run_yolo_face_track.sh`
  - `F:\小车文件\face_tools\run_face_follow_stack.sh`

### 5. ONNX 人脸模型已下载并验证
- 本地模型:
  - `F:\小车文件\face_tools\models\face_detection_yunet_2023mar.onnx`
  - `F:\小车文件\face_tools\models\face_recognition_sface_2021dec.onnx`
- 本地验证结果:
  - OpenCV `FaceDetectorYN_create` 成功
  - OpenCV `FaceRecognizerSF_create` 成功

### 6. 本地人脸录入已完成
- 已使用本机前置摄像头录入 `owner`
- 路径: `F:\小车文件\face_tools\captured_faces\owner`
- 当前累计样本: 40 张

### 7. 本地 ONNX 人脸库构建与识别验证完成
- 本地库:
  - `F:\小车文件\face_tools\captured_faces\local_embeddings.npz`
- 构建结果:
  - backend = `sface`
  - `owner: 40 valid images`
- 识别抽检:
  - 对 `owner` 样本图匹配得分约 `0.8151`
- 说明本地 ONNX 人脸识别链路已经可用

### 8. 追踪控制层已补上
- 新增“识别脚本 -> 状态文件 -> ROS2 跟随控制节点”链路
- 目的:
  - 避免把 ACL/CANN 与 ROS2 强耦合到同一个 Python 环境
  - 提高调试和运行稳定性
- 新文件:
  - `F:\小车文件\face_tools\face_follow_controller.py`
  - `F:\小车文件\face_tools\run_face_follow_stack.sh`
  - `F:\小车文件\face_tools\run_follow_demo_full_stack.sh`
  - `F:\小车文件\face_tools\run_intruder_follow_demo_full_stack.sh`
- 行为:
  - `yolo_face_track_alarm.py` 会持续写 `/tmp/face_follow_state.json`
  - `face_follow_controller.py` 读取该状态并发布 `/car_cmd_vel`
  - 状态超时会自动回中位安全值 `1500/90`
  - 默认目标名为 `owner`
  - 已支持 `follow-policy`:
    - `owner`: 跟随指定已录入人
    - `named`: 跟随任意已识别人
    - `unknown`: 跟随陌生人
    - `none`: 只识别不跟随

### 9. 小车端已同步新的跟随脚本
- 已同步到:
  - `/root/face_tools/face_follow_controller.py`
  - `/root/face_tools/run_face_follow_stack.sh`
  - `/root/face_tools/run_follow_demo_full_stack.sh`
  - `/root/face_tools/run_intruder_follow_demo_full_stack.sh`
  - `/root/face_tools/capture_face_on_car.py`
- 已验证:
  - `face_follow_controller.py --help` 正常
  - 人工状态输入时，控制器日志能计算出非中位控制量
  - 保险逻辑有效: 状态超时会自动回 `1500/90`
  - 当前小车端识别脚本已支持 `sface` 后端和陌生人跟随策略
  - `unknown` 模式的控制器人工状态测试已确认会输出非中位控制量

### 10. 本地尚未再次同步到小车的最新修复
- 以下修复已经在本地完成, 但由于 `192.168.5.100` 网络再次断开, 尚未完成最后一次同步:
  - `yolo_face_track_alarm.py`
    - 改为“全帧先找脸, 再关联到人框”, 识别稳定性更好
  - `run_face_follow_stack.sh`
    - 恢复录像保存询问
    - `owner` 跟随链改为由识别脚本决定 active 目标, 控制器只跟 active
  - `run_follow_demo_full_stack.sh`
    - 启动后自动关闭被激光雷达驱动拉起的 RViz
  - `run_intruder_follow_demo_full_stack.sh`
    - 同样自动关闭 RViz
  - `face_follow_controller.py`
    - 新增 `target_mode`
    - 默认速度/转向参数更积极, 更容易让车动起来

### 11. 当前已确认的真实问题点
- “为什么会打开 RViz”
  - 根因不是用户误操作
  - 是 `/home/racecar/src/racecar/launch/Run_car.launch.py` 里包含的 `lslidar_launch.py`
  - `lslidar_launch.py` 默认直接启动 `rviz2`
- “为什么视频没问保存”
  - 根因是之前为了提速, `run_face_follow_stack.sh` 默认强制加了 `--no-record`
  - 本地修复已移除该强制项, 待下次同步到小车
- “为什么识别到了但不跟随”
  - 一个根因是陌生人跟随链原先把控制器写死成只认 `owner`
  - 本地修复已改成:
    - `yolo_face_track_alarm.py` 负责决定 active 目标
    - 控制器只负责跟 active 目标
  - 另一个根因是早期速度参数过于保守
  - 本地修复已提高默认最小速度与转向增益

## 当前阻塞
- 小车 SSH 当前不可达
- 症状:
  - `ssh root@192.168.5.100` 超时
  - `ping 192.168.5.100` 丢包 100%
- 这不是代码逻辑错误, 更像网络链路/设备上电/网卡状态问题

## 与小车相关的关键已知信息

### 硬件
- Atlas 200 DK (c78)
- Ascend310B
- 1.8GB 内存
- 文档显示 `RealSense D435i`
- RPLIDAR X4
- 小车 IP: `192.168.5.100`

### 硬件注意
- 用户口头反馈“当前相机不一定是深度相机”
- 但硬件文档 `E:\小车\小车硬件数据\昇腾智能小车硬件参数详细文档.md` 明确写的是 `RealSense D435i`
- 后续要通过运行时话题和设备节点再次确认:
  - 是否真的有深度流
  - 当前导航/识别程序是否只使用 RGB

### 车控
- 关键控制话题: `/car_cmd_vel`
- 关键实现文件:
  - `/home/racecar/src/racecar/src/car_controller_new.cpp`
- 已确认语义:
  - 中位速度: `linear.x = 1500`
  - 中位转向: `angular.z = 90`
  - 转向限幅: `45 ~ 135`

## 追踪项目参考结论
- 本地参考工程:
  - `E:\小车\昇腾追踪项目完整代码\object_tracking_video`
- 结论:
  - 适合做 Ascend 跟踪算法参考
  - 不适合直接当实体车主程序
- 原因:
  - 偏离线视频处理
  - 缺少 ROS2 车控闭环
  - 缺少人脸库/识别/告警联动

## 当前最重要的文件

### 小车端
- `/root/yolo_om_cam_bbox.py`
- `/root/run_yolo_bbox.sh`
- `/root/yolov5s_310b.om`
- `/root/face_tools/yolo_face_track_alarm.py`
- `/root/face_tools/face_follow_controller.py`
- `/root/face_tools/run_face_follow_stack.sh`
- `/root/face_tools/run_follow_demo_full_stack.sh`
- `/root/face_tools/run_intruder_follow_demo_full_stack.sh`
- `/root/face_tools/capture_face_on_car.py`
- `/root/alarm_atlas.py`
- `/root/combo_alarm.py`
- `/home/racecar/src/racecar/src/car_controller_new.cpp`
- `/home/racecar/src/racecar/scripts/detct_headless.py`
- `/home/racecar/nav.sh`
- `/home/racecar/src/racecar/launch/classroom_nav.launch.py`
- `/home/racecar/src/racecar/config/nav/navigation.yaml`

### 本地端
- `F:\小车文件\face_tools\enroll_face_and_sync.py`
- `F:\小车文件\face_tools\sync_face_assets_to_car.py`
- `F:\小车文件\face_tools\build_face_db.py`
- `F:\小车文件\face_tools\yolo_face_track_alarm.py`
- `F:\小车文件\face_tools\models\face_detection_yunet_2023mar.onnx`
- `F:\小车文件\face_tools\models\face_recognition_sface_2021dec.onnx`
- `F:\小车文件\face_tools\captured_faces\owner`

## 一旦 SSH 恢复, 下一步直接执行

### 1. 同步人脸模型、脚本、已录入样本到小车
```powershell
python "F:\小车文件\face_tools\sync_face_assets_to_car.py" --person owner
```

### 2. 在小车上运行“检测 + TrackID + 人脸识别 + 陌生人告警”
```bash
/root/run_yolo_face_track.sh --infer-interval 2 --face-interval 6 --record-step 2 --print-every 20
```

### 3. 在小车上运行“识别 + 跟随控制”联动栈
```bash
/root/face_tools/run_face_follow_stack.sh --infer-interval 2 --face-interval 6 --print-every 20 --no-show
```

### 4. 在小车上运行“完整跟随演示（含底盘驱动）”
```bash
/root/face_tools/run_follow_demo_full_stack.sh --infer-interval 2 --face-interval 6 --print-every 20
```

### 5. 在小车上运行“陌生人跟随演示（含底盘驱动）”
```bash
/root/face_tools/run_intruder_follow_demo_full_stack.sh --infer-interval 2 --face-interval 6 --print-every 20
```

### 6. 验证
- 画面上应出现:
  - `idX:owner`
  - 未录入人员显示 `unknown`
- 当 `unknown` 连续达到阈值时触发报警命令
- 若底盘驱动节点在线，则跟随控制节点会持续发 `/car_cmd_vel`
- 注意: 不要与其他会发布 `/car_cmd_vel` 的巡线/控制脚本同时运行

## 巡航与安防项目的新目标方案

### 总体状态机
- `PATROL`: 按地图与固定路线巡航
- `ALERT_TRACK`: 巡航过程中发现陌生人, 暂停巡航并转入跟踪
- `RETURN_TO_ROUTE`: 陌生人离开警戒范围或跟踪结束, 返回最近巡航点继续任务
- `SAFE_STOP`: 目标丢失/冲突/系统异常时停车

### 推荐实现拆分
1. `Nav2 Patrol Manager`
- 负责在地图上发送固定巡航点
- 参考 `/home/racecar/nav.sh` 和 `classroom_nav.launch.py`
- 用 `NavigateToPose` 或 `NavigateThroughPoses`

2. `Face Security Monitor`
- 负责检测、人脸识别、陌生人判定、写目标状态
- 当前基础已经有: `/root/face_tools/yolo_face_track_alarm.py`

3. `Intruder Follow Controller`
- 负责在 `ALERT_TRACK` 状态下根据陌生人框中心和距离控制车体
- 当前基础已经有: `/root/face_tools/face_follow_controller.py`

4. `Patrol Supervisor`
- 负责状态切换:
  - 巡航 -> 发现陌生人 -> 暂停导航
  - 跟踪结束 -> 回到最近航点 -> 继续巡航

### 避障方案
- 现有 Nav2 配置已经接入激光雷达代价地图
- 关键配置文件:
  - `/home/racecar/src/racecar/config/nav/navigation.yaml`
- 已确认:
  - local/global costmap 都使用 `/scan`
  - obstacle layer 已启用
- 结论:
  - 巡航阶段避障已有基础
  - 跟踪阶段若直接发 `/car_cmd_vel`，则需要额外安全约束
  - `nav.sh` / Nav2 / `car_controller_new` 这一链本身也会控制 `/car_cmd_vel`
  - 所以最终比赛版不能让“导航控制器”和“人脸跟随控制器”长期同时抢 `/car_cmd_vel`
  - 更稳的下一版是把“跟踪目标”转成 Nav2 短目标点，而不是长期裸发 `/car_cmd_vel`

### 当前最合理的工程路线
1. 先把“识别 + 陌生人跟随”单独跑稳
2. 再做“巡航状态下发现陌生人时中断 Nav2”
3. 再做“离开范围后回到最近航点继续巡航”
4. 最后再处理多人冲突策略
5. 最终统一控制权, 避免多个节点长期同时发布 `/car_cmd_vel`

## 后续必须继续做的工作

### A. 小车追踪闭环
- 当前已经有一个最小可运行版“目标选择 + 控制器”:
  - 输入: `/tmp/face_follow_state.json`
  - 输出: `/car_cmd_vel`
- 控制策略建议:
  - 横向误差 `ex = target_cx - image_center_x`
  - `angular.z = 90 + Kp * ex`
  - 目标远近优先用 D435 深度控制 `linear.x`
  - 丢目标或陌生人报警时回中位 `1500/90`
- 下一版建议:
  - 不再只用框面积估计距离
  - 直接接 D435 深度信息做稳态距离控制
  - 增加速度/角度平滑器和加速度限制

### B. 巡航与回线
- 新建 `patrol_supervisor.py`
- 负责:
  - 下发巡航航点
  - 监听陌生人事件
  - 暂停/恢复 Nav2 任务
  - 记录当前最近航点
- 不建议复用当前 `navigation_test.py` 源文件
  - 该源文件内容可疑, 需要单独重写更稳

### C. 识别与追踪冲突策略
- 当前先不做多人冲突
- 后续建议:
  - 陌生人优先级 > owner
  - 同时存在多个陌生人时按:
    - 最近距离
    - 最大框面积
    - 连续出现时间
  选择一个主目标

### D. 显示与调试
- 当前“没画面”通常由以下原因造成:
  - 用了 `--no-show`
  - SSH 会话没有 X11 转发
  - MobaXterm X server 未启用
- 代码层面目前默认支持显示
- 若要车动起来, 还必须启动底盘驱动
  - 可直接用 `/root/face_tools/run_follow_demo_full_stack.sh`

### B. 性能优化
- 当前瓶颈主要来自:
  - 640 输入推理
  - 每帧后处理
  - 摄像头 + 可视化 + 录像写盘
- 建议优化:
  - 只检测人类 `target-cls=0`
  - 检测每 2~3 帧执行一次
  - 人脸识别每 6~10 帧执行一次
  - 日志降频
  - 录像按需写盘

### C. 人脸识别精度优化
- 每个人录 20~50 张
- 多角度、多光照
- 陌生人阈值需要实测微调
- 如果比赛环境光照差, 建议录制室内外两套样本

## 已知注意事项
- Windows 下 OpenCV 读取中文路径 ONNX 可能失败
- 已在 `enroll_face_and_sync.py` 中兼容:
  - 自动把模型复制到英文临时目录再加载
- 小车侧路径是英文路径, 不受这个问题影响

## 给下一个 AI 的最短接手说明
- 先别重复安装 CANN, 它已经好了
- 别重复做 ONNX -> OM, `/root/yolov5s_310b.om` 已经成功
- 先解决 SSH/网络恢复
- 网络恢复后直接跑:
  - `python "F:\小车文件\face_tools\sync_face_assets_to_car.py" --person owner`
- 再在小车上跑:
  - `/root/face_tools/run_face_follow_stack.sh --infer-interval 2 --face-interval 6 --print-every 20 --no-show`
- 当前已经有最小追踪闭环, 下一步重点是:
  - 深度相机距离控制
  - 控制参数实车标定
  - 多发布者冲突规避
  - 巡航中断与返回原路线状态机
