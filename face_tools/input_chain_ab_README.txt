输入链 A/B 定责使用说明

1. 首次执行，先把“当前坏现象版本”保存为 A：
   bash /root/face_tools/input_chain_ab_switch.sh capture-current

2. 查看当前状态：
   bash /root/face_tools/input_chain_ab_switch.sh status

3. 切换实验版本：
   bash /root/face_tools/input_chain_ab_switch.sh activate A
   bash /root/face_tools/input_chain_ab_switch.sh activate B
   bash /root/face_tools/input_chain_ab_switch.sh activate C
   bash /root/face_tools/input_chain_ab_switch.sh activate D
   bash /root/face_tools/input_chain_ab_switch.sh activate E
   bash /root/face_tools/input_chain_ab_switch.sh activate F

4. 含义：
   A = 当前 encoder + 当前 imu_encoder_mix + 当前 odom_tf
   B = 旧 encoder + 当前 imu_encoder_mix + 当前 odom_tf
   C = 当前 encoder + 旧 imu_encoder_mix + 当前 odom_tf
   D = 旧 encoder + 旧 imu_encoder_mix + 当前 odom_tf
   E = 当前 encoder + 当前 imu_encoder_mix + 旧 odom_tf
   F = 旧 encoder + 旧 imu_encoder_mix + 旧 odom_tf

5. 旧版本固定指向：
   *.bak_codex_20260408_input_chain_round2

6. 每次切完版本后，按同一条走廊、同一起点、同一驾驶方式复现；
   停止建图后执行：
   bash /root/face_tools/collect_mapping_evidence.sh A_run1 A

7. 重点看 summary.txt 里的 5 类证据：
   - Write timeout
   - encoder input unstable / encoder sample dt abnormal
   - odom_tf 是否长时间卡住
   - slam_gmapping 是否出现大跳 update / queue is full / Scan Matching Failed
   - 最终地图是否仍然扭曲
