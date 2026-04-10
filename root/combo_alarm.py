#!/usr/bin/env python3
import sys
import time
import subprocess
import threading
from audio_alarm import AudioAlarm
from led_alarm import led_alarm

class ComboAlarm:
    def __init__(self):
        self.audio = AudioAlarm()

    def alarm(self, duration=3):
        """组合报警：音频 + LED + 屏幕闪烁"""
        print(f"🚨🚨🚨 组合报警系统启动！持续 {duration} 秒 🚨🚨🚨")

        # 启动音频报警
        audio_thread = threading.Thread(target=self.audio.alarm, args=(duration,))
        audio_thread.start()

        # LED报警
        led_thread = threading.Thread(target=led_alarm, args=(duration,))
        led_thread.start()

        # 屏幕闪烁
        for i in range(int(duration)):
            print("\n" + "=" * 50)
            print("⚠️  警告！小车报警系统激活！ ⚠️")
            print("=" * 50 + "\n", end='\r')
            time.sleep(1)

        # 等待其他线程
        audio_thread.join()
        led_thread.join()

        print("\n✅ 报警系统完成\n")

def main():
    if len(sys.argv) > 1:
        duration = float(sys.argv[1])
    else:
        duration = 3

    alarm = ComboAlarm()
    alarm.alarm(duration)

if __name__ == '__main__':
    main()
