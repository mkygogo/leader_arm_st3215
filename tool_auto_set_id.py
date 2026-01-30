import time
import sys
from sts3215_driver import STSServoDriver

# ================= 配置 =================
PORT = '/dev/ttyACM0'  # Linux端口
BAUDRATE = 1000000
# =======================================

def main():
    print("========================================================")
    print("        Feetech STS Servo Auto-ID Configurator          ")
    print("========================================================")
    
    try:
        sts = STSServoDriver(PORT, baudrate=BAUDRATE)
    except Exception as e:
        print(f"Error opening serial port: {e}")
        return

    print("Step 1: Scanning for connected servo...")
    print("PLEASE WAIT (Scanning ID 0-253)...")
    
    found_ids = []
    
    # 快速扫描所有可能的ID
    # 为了加快速度，我们可以暂时把超时设短一点，或者直接复用ping
    # 250个ID扫描大概需要几秒钟
    for scan_id in range(254):
        # 每扫描10个ID打印一个点，作为进度条
        if scan_id % 10 == 0:
            print(".", end="", flush=True)
            
        if sts.ping(scan_id):
            found_ids.append(scan_id)
            # 如果只想找一个，找到就可以跳出，但为了安全建议扫完，防止连了两个
            # break 

    print("\n")

    # ================= 结果分析 =================
    if len(found_ids) == 0:
        print("❌ No servos found!")
        print("   - Check power (6V-12V)")
        print("   - Check connections (RX/TX)")
        print("   - Check Baudrate")
        sts.close()
        return

    if len(found_ids) > 1:
        print(f"❌ DANGER: Multiple servos detected! IDs: {found_ids}")
        print("   To prevent ID conflicts, please connect ONLY ONE servo at a time.")
        print("   Disconnect the others and run this tool again.")
        sts.close()
        return

    # 只有一个舵机，安全
    current_id = found_ids[0]
    print(f"✅ Found ONE servo at ID: [ {current_id} ]")

    # ================= 修改 ID =================
    try:
        raw_new = input(f"Enter NEW ID for this servo (Current: {current_id}) > ")
        new_id = int(raw_new)
    except ValueError:
        print("Invalid input.")
        sts.close()
        return

    if new_id < 0 or new_id > 253:
        print("Error: ID must be between 0 and 253.")
        sts.close()
        return

    if new_id == current_id:
        print("New ID is the same as current ID. Done.")
        sts.close()
        return

    print(f"\nSetting ID from {current_id} to {new_id}...")
    sts.change_id(current_id, new_id)
    time.sleep(0.5)

    # ================= 验证 =================
    print(f"Verifying ID {new_id}...")
    if sts.ping(new_id):
        print(f"🎉 SUCCESS! Servo ID successfully changed to {new_id}")
    else:
        print(f"❌ FAILED. Servo not responding at {new_id}. It might still be {current_id}.")

    sts.close()

if __name__ == "__main__":
    main()