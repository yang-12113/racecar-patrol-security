#!/usr/bin/env python3
import time
import subprocess
import sys

def led_alarm(duration=3):
    """使用系统状态LED进行报警"""
    print(f"💡 LED报警已触发！持续 {duration} 秒")

    # 尝试控制LED
    try:
        # 方法1: 使用系统LED
        for i in range(int(duration * 2)):
            # 打开LED
            subprocess.run(['echo', '1'], stdout=subprocess.PIPE,
                         input='heartbeat', text=True)
            time.sleep(0.5)
            # 关闭LED
            subprocess.run(['echo', '0'], stdout=subprocess.PIPE,
                         input='none', text=True)
            time.sleep(0.5)
    except:
        # 方法2: 屏幕闪烁
        for i in range(int(duration)):
            print("\033[5m" + "🚨 报警！🚨" + "\033[0m", end='\r')
            time.sleep(0.5)
            print("                   ", end='\r')
            time.sleep(0.5)

def main():
    if len(sys.argv) > 1:
        duration = float(sys.argv[1])
    else:
        duration = 3

    led_alarm(duration)

if __name__ == '__main__':
    main()
