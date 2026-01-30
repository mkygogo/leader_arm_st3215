import time
import sys
from sts3215_driver import STSServoDriver

# ================= 配置 =================
PORT = '/dev/ttyACM0'  # 你的端口
BAUDRATE = 1000000
# =======================================

def main():
    print("========================================================")
    print("           Feetech STS Servo ID Changer Tool            ")
    print("========================================================")
    print("WARNING: Only connect the ONE servo you want to configure.")
    print("Do NOT connect multiple servos with the same ID together.")
    print("========================================================\n")

    try:
        sts = STSServoDriver(PORT, baudrate=BAUDRATE)
    except Exception as e:
        print(f"Error opening serial port: {e}")
        return

    # 1. 输入旧 ID
    try:
        raw_old = input("Enter current Servo ID (default is usually 1): ")
        old_id = int(raw_old)
    except ValueError:
        print("Invalid input.")
        return

    # 2. 检查旧 ID 是否存在
    print(f"\nChecking for servo at ID {old_id}...")
    if not sts.ping(old_id):
        print(f"❌ Error: No servo found at ID {old_id}.")
        print("Please check wiring and power.")
        sts.close()
        return
    else:
        print(f"✅ Found servo at ID {old_id}.")

    # 3. 输入新 ID
    try:
        raw_new = input(f"Enter NEW Servo ID (e.g., 7 for gripper): ")
        new_id = int(raw_new)
    except ValueError:
        print("Invalid input.")
        return

    if new_id < 0 or new_id > 253:
        print("Error: ID must be between 0 and 253.")
        return

    if new_id == old_id:
        print("Old ID and New ID are the same. Nothing to do.")
        return

    # 4. 执行修改
    print(f"\nChanging ID from {old_id} to {new_id}...")
    sts.change_id(old_id, new_id)
    
    # 给一点时间让EEPROM写入生效
    time.sleep(0.5)

    # 5. 验证结果
    print(f"Verifying new ID {new_id}...")
    if sts.ping(new_id):
        print(f"✅ SUCCESS! Servo ID is now {new_id}.")
        
        # 顺便读一下位置确认通讯正常
        pos = sts.get_position(new_id)
        print(f"Current Position: {pos}")
    else:
        print(f"❌ Failed. Servo did not respond at new ID {new_id}.")
        print("Try scanning to see if the ID remained at the old value.")

    sts.close()

if __name__ == "__main__":
    main()