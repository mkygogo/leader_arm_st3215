import time
import sys
from sts3215_driver import STSServoDriver

# ================= 配置 =================
PORT = '/dev/ttyACM0'
BAUDRATE = 1000000
SERVO_IDS = [1, 2, 3, 4, 5, 6, 7]  # 你的6个关节ID
# =======================================

def main():
    # 初始化驱动
    try:
        sts = STSServoDriver(PORT, baudrate=BAUDRATE)
        print(f"Connected to {PORT} at {BAUDRATE}bps")
    except Exception as e:
        print(f"Failed to connect: {e}")
        return

    print("================================================================")
    print(f"Monitoring Servos: {SERVO_IDS}")
    print("Press Ctrl+C to stop")
    print("================================================================\n")

    try:
        while True:
            output_str = ""
            
            # 遍历读取每个舵机的状态
            for servo_id in SERVO_IDS:
                # 只读取位置 (get_position返回 0-4095)
                pos = sts.get_position(servo_id)
                
                if pos != -1:
                    # 格式化输出: ID1:2048
                    # :<4d 表示占4位并左对齐，保持界面整洁
                    output_str += f"ID{servo_id}:{pos:<4d}  " 
                else:
                    # 如果读不到，显示 ERR
                    output_str += f"ID{servo_id}:ERR   "

            # \r 表示回到行首，end='' 表示不换行，flush=True 强制刷新缓冲区
            print(f"\r{output_str}", end="", flush=True)
            
            # 适当延时，太快了串口可能堵塞，也看不清
            # 6个舵机每次轮询大概需要 10ms-20ms 左右的物理传输时间
            time.sleep(0.05) 

    except KeyboardInterrupt:
        print("\n\nStopped.")
        sts.close()

if __name__ == "__main__":
    main()