#!/usr/bin/env python3
import os
import time

class GPIOBuzzer:
    def __init__(self, gpio_pin=18):
        self.gpio_pin = gpio_pin
        self.export_gpio()
        
    def export_gpio(self):
        try:
            with open("/sys/class/gpio/export", "w") as f:
                f.write(str(self.gpio_pin))
        except IOError:
            pass
        with open(f"/sys/class/gpio/gpio{self.gpio_pin}/direction", "w") as f:
            f.write("out")
    
    def set_value(self, value):
        with open(f"/sys/class/gpio/gpio{self.gpio_pin}/value", "w") as f:
            f.write(str(value))
    
    def beep(self, duration=1.0):
        self.set_value(1)
        time.sleep(duration)
        self.set_value(0)
    
    def cleanup(self):
        try:
            with open("/sys/class/gpio/unexport", "w") as f:
                f.write(str(self.gpio_pin))
        except IOError:
            pass

if __name__ == "__main__":
    buzzer = GPIOBuzzer(18)
    print("触发蜂鸣器...")
    buzzer.beep(1.0)
    buzzer.cleanup()
    print("测试完成")
