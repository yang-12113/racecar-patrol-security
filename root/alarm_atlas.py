#!/usr/bin/env python3
import sys
import os

# 导入组合报警系统
sys.path.append('/root')
from combo_alarm import ComboAlarm

def main():
    if len(sys.argv) > 1:
        duration = float(sys.argv[1])
    else:
        duration = 3

    alarm = ComboAlarm()
    alarm.alarm(duration)

if __name__ == '__main__':
    main()
