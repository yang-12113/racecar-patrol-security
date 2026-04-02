#!/usr/bin/env python3

import os
import subprocess
import time

def main():
    print(Starting小车报警系统...)
    
    # 设置ROS2环境
    os.chdir('/home/racecar')
    subprocess.run(['source', '/opt/ros/humble/setup.bash'], shell=True)
    subprocess.run(['source', 'install/setup.bash'], shell=True)
    
    # 启动报警节点
    print(启动报警节点...)
    alarm_process = subprocess.Popen([
        'python3', '/home/racecar/install/racecar/lib/racecar/alarm_node_simple.py'
    ])
    
    time.sleep(2)
    
    # 启动报警监控
    print(启动报警监控...)
    monitor_process = subprocess.Popen([
        'python3', '/home/racecar/install/racecar/lib/racecar/alarm_monitor.py'
    ])
    
    print(报警系统已启动!)
    print(报警节点PID:, alarm_process.pid)
    print(监控节点PID:, monitor_process.pid)
    print(按Ctrl+C停止系统...)
    
    try:
        alarm_process.wait()
        monitor_process.wait()
    except KeyboardInterrupt:
        print(n停止报警系统...)
        alarm_process.terminate()
        monitor_process.terminate()
        alarm_process.wait()
        monitor_process.wait()
        print(报警系统已停止)

if __name__ == '__main__':
    main()
