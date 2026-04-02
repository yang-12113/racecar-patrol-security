#!/usr/bin/env python3

import os
import subprocess
import time

def test_sound():
    print("Testing alarm sounds...")
    
    # 方法1: 终端铃声
    print("1. Terminal bell:")
    os.system('echo -e "\a"')
    time.sleep(1)
    
    # 方法2: 控制台铃声
    print("2. Console bell:")
    os.system('printf "\a"')
    time.sleep(1)
    
    # 方法3: 尝试speaker-test
    print("3. Speaker test:")
    try:
        result = subprocess.run(['speaker-test', '-t', 'sine', '-f', '1000', '-l', '200'], 
                              capture_output=True, timeout=3, text=True)
        if result.returncode == 0:
            print("Speaker test successful")
        else:
            print("Speaker test failed:", result.stderr)
    except FileNotFoundError:
        print("speaker-test not found")
    except Exception as e:
        print("Speaker test error:", e)
    
    print("Sound test completed!")

if __name__ == '__main__':
    test_sound()
