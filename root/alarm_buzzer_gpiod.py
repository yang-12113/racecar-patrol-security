import gpiod
import time

# 配置蜂鸣器GPIO口（昇腾板的GPIO编号，根据实际接线修改）
CHIP = "gpiochip0"
BUZZER_LINE = 127  # 示例编号，请替换为你的实际引脚

def alarm():
    print("小车报警已触发！蜂鸣器将鸣响3秒")
    chip = gpiod.Chip(CHIP)
    line = chip.get_line(BUZZER_LINE)
    line.request(consumer="buzzer", type=gpiod.LINE_REQ_DIR_OUT)
    line.set_value(1)  # 蜂鸣器响
    time.sleep(3)
    line.set_value(0)  # 蜂鸣器停
    line.release()
    chip.close()

if __name__ == "__main__":
    alarm()
