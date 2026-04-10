import RPi.GPIO as GPIO
import time

# 配置蜂鸣器GPIO口（请根据你的小车实际接线修改，这里默认BCM模式18口）
BUZZER_PIN = 18
GPIO.setmode(GPIO.BCM)
GPIO.setup(BUZZER_PIN, GPIO.OUT)

def alarm():
    print("小车报警已触发！蜂鸣器将鸣响3秒")
    GPIO.output(BUZZER_PIN, GPIO.HIGH)
    time.sleep(3)
    GPIO.output(BUZZER_PIN, GPIO.LOW)
    GPIO.cleanup()

if __name__ == "__main__":
    alarm()
