#!/usr/bin/env python3
import os
import time

def check_gpio_availability():
    print("检查GPIO可用性...")

    # 检查每个GPIO芯片
    chips = ['gpiochip0', 'gpiochip32', 'gpiochip128', 'gpiochip160', 'gpiochip224']

    for chip in chips:
        try:
            with open(f'/sys/class/gpio/{chip}/ngpio', 'r') as f:
                num_pins = int(f.read().strip())
                print(f"{chip}: {num_pins} 个引脚")

                # 计算实际的GPIO编号
                if chip == 'gpiochip0':
                    base = 0
                elif chip == 'gpiochip32':
                    base = 32
                elif chip == 'gpiochip128':
                    base = 128
                elif chip == 'gpiochip160':
                    base = 160
                elif chip == 'gpiochip224':
                    base = 224

                # 尝试几个引脚
                for offset in [0, 1, 2, 3, 4]:
                    gpio_num = base + offset
                    try:
                        # 尝试导出GPIO
                        with open('/sys/class/gpio/export', 'w') as f:
                            f.write(str(gpio_num))

                        # 检查是否成功
                        if os.path.exists(f'/sys/class/gpio/gpio{gpio_num}'):
                            print(f"  ✓ GPIO {gpio_num} 可用")
                            # 立即释放
                            with open('/sys/class/gpio/unexport', 'w') as f:
                                f.write(str(gpio_num))
                    except:
                        pass

        except Exception as e:
            print(f"{chip}: 错误 - {str(e)}")

if __name__ == '__main__':
    check_gpio_availability()
