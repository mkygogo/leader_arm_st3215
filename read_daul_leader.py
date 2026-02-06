import time
import sys
import serial.tools.list_ports
from leader_arm_st3215 import LeaderArm

# ================= 请填入你修正后的配置 =================
LEADER_CFG = {
    'left':  '5A68012049',   # 填 Serial 或 Location
    'right': '1-2:1.0'       # 填 Serial 或 Location (别填重复的Serial!)
}
# ======================================================

def find_port(identifier):
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        # 匹配 Serial
        if p.serial_number == identifier:
            return p.device
        # 匹配 Location
        if p.location and identifier in p.location: # 部分匹配
            return p.device
    return None

def main():
    print("Checking Leader Arms connection...")
    
    arms = {}
    for side, ident in LEADER_CFG.items():
        port = find_port(ident)
        if port:
            print(f"✅ Found {side} leader at {port} (ID: {ident})")
            try:
                # 尝试连接
                arm = LeaderArm(port, config_file=f"leader_{side}_config.json")
                arms[side] = arm
            except Exception as e:
                print(f"❌ Failed to open {side}: {e}")
        else:
            print(f"❌ Could not find device for {side} (ID: {ident})")

    if len(arms) < 2:
        print("\n⚠️ Warning: Not all arms connected.")
    
    print("\nReading data (Ctrl+C to stop)...")
    try:
        while True:
            out_str = ""
            for side in ['left', 'right']:
                if side in arms:
                    # 只读第1个关节简单验证
                    pos = arms[side].driver.get_position(1)
                    val = f"{pos}" if pos != -1 else "ERR"
                    out_str += f"[{side.upper()} J1: {val}]  "
                else:
                    out_str += f"[{side.upper()} --]  "
            
            print(f"\r{out_str}", end="", flush=True)
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        pass
    finally:
        for arm in arms.values():
            arm.driver.close()

if __name__ == "__main__":
    main()