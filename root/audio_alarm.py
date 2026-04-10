#!/usr/bin/env python3
import os
import sys
import time
import subprocess
import threading

class AudioAlarm:
    def __init__(self):
        self.alarm_active = False

    def beep(self, frequency=1000, duration=1.0):
        """使用系统音频发出蜂鸣声"""
        try:
            # 方法1: 使用beep命令
            subprocess.run(['beep', '-f', str(frequency), '-l', str(int(duration*1000))],
                         capture_output=True, timeout=duration+1)
            return True
        except:
            pass

        try:
            # 方法2: 使用speaker-test
            subprocess.run(['speaker-test', '-t', 'sine', '-f', str(frequency), '-l', '1'],
                         capture_output=True, timeout=duration+1)
            return True
        except:
            pass

        try:
            # 方法3: 使用aplay播放生成的音频
            # 生成一个简单的WAV文件
            wav_data = self.generate_sine_wave(frequency, duration)
            with open('/tmp/alarm.wav', 'wb') as f:
                f.write(wav_data)
            subprocess.run(['aplay', '/tmp/alarm.wav'],
                         capture_output=True, timeout=duration+1)
            os.remove('/tmp/alarm.wav')
            return True
        except:
            pass

        # 方法4: 终端铃声
        print('\a' * 10)
        return False

    def generate_sine_wave(self, frequency, duration):
        """生成正弦波WAV数据"""
        import wave
        import struct
        import math

        sample_rate = 44100
        samples = int(sample_rate * duration)

        # WAV文件头
        wav_header = struct.pack('<4sL4s', b'RIFF', 36 + samples * 2, b'WAVE')
        fmt_chunk = struct.pack('<4sLHHLLHH', b'fmt ', 16, 1, 1, sample_rate,
                               sample_rate * 2, 2, 16)
        data_chunk = struct.pack('<4sL', b'data', samples * 2)

        # 生成音频数据
        audio_data = b''
        for i in range(samples):
            value = int(32767 * math.sin(2 * math.pi * frequency * i / sample_rate))
            audio_data += struct.pack('<h', value)

        return wav_header + fmt_chunk + data_chunk + audio_data

    def alarm(self, duration=3):
        """触发报警"""
        if self.alarm_active:
            return

        self.alarm_active = True
        print(f"🚨 音频报警已触发！持续 {duration} 秒")

        # 多线程播放多个声音
        threads = []

        # 主蜂鸣声
        t1 = threading.Thread(target=self.beep, args=(1000, duration))
        t1.start()
        threads.append(t1)

        # 高频警告声
        for i in range(int(duration)):
            t2 = threading.Thread(target=self.beep, args=(2000, 0.2))
            t2.start()
            time.sleep(0.5)

        # 等待所有线程完成
        for t in threads:
            t.join()

        self.alarm_active = False

def main():
    if len(sys.argv) > 1:
        duration = float(sys.argv[1])
    else:
        duration = 3

    alarm = AudioAlarm()
    alarm.alarm(duration)

if __name__ == '__main__':
    main()
