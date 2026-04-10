#!/bin/bash
# 配置蜂鸣器GPIO口（根据实际接线修改，示例为GPIO127）
BUZZER_PIN=127

# 导出GPIO口
echo $BUZZER_PIN > /sys/class/gpio/export
# 设置为输出模式
echo out > /sys/class/gpio/gpio$BUZZER_PIN/direction

echo "小车报警已触发！蜂鸣器将鸣响3秒"
# 蜂鸣器响
echo 1 > /sys/class/gpio/gpio$BUZZER_PIN/value
sleep 3
# 蜂鸣器停
echo 0 > /sys/class/gpio/gpio$BUZZER_PIN/value

# 释放GPIO口
echo $BUZZER_PIN > /sys/class/gpio/unexport
