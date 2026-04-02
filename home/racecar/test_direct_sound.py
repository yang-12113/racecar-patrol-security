#!/usr/bin/env python3

import os
import time
import subprocess

def test_all_sounds():
    print("=== 直接声音测试 ===")
    
    # 测试1: printf bell
    print("1. 测试printf铃声:")
    for i in range(3):
        os.system('printf "\a"')
        time.sleep(0.5)
    
    print("\n2. 测试echo铃声:")
    for i in range(3):
        os.system('echo -e "\a"')
        time.sleep(0.5)
    
    print("\n3. 测试tput铃声:")
    try:
        subprocess.run(['tput', 'bel'], capture_output=True)
        time.sleep(0.5)
        subprocess.run(['tput', 'bel'], capture_output=True)
        print("tput铃声测试完成")
    except:
        print("tput不可用")
    
    print("\n4. 测试直接字符:")
    for i in range(3):
        print('\a', end='', flush=True)
        time.sleep(0.5)
    print()
    
    print("=== 声音测试完成 ===")

if __name__ == '__main__':
    test_all_sounds()
