import time
import json
import os
from sts3215_driver import STSServoDriver

# 默认配置
DEFAULT_CONFIG_FILE = "leader_config.json"
STS_RESOLUTION = 4096.0
STS_DEGREE_RANGE = 360.0

class LeaderArm:
    def __init__(self, port, baudrate=1000000, servo_ids=[1, 2, 3, 4, 5, 6, 7]):
        """
        Leader Arm (主臂) 控制类
        负责读取 STS3215 舵机数据并转换为角度
        """
        self.servo_ids = servo_ids
        self.driver = STSServoDriver(port, baudrate)
        
        # 默认零点偏移量 (默认为2048，即舵机中间位置)
        self.home_offsets = {id: 2048 for id in servo_ids}
        
        # 默认方向系数 (1 或 -1)，用于后续反向修正
        self.directions = {id: 1 for id in servo_ids}
        
        # 尝试加载配置文件
        self.load_config()

        print("Leader Arm Initialized.")
        print(f"IDs: {self.servo_ids}")
        
    def set_torque(self, enable):
        """
        设置主臂力矩
        enable=False: 卸力模式 (人手控制)
        enable=True:  保持模式 (机器锁死)
        """
        state = "ON" if enable else "OFF"
        print(f"Setting Leader Arm Torque -> {state}")
        for sid in self.servo_ids:
            self.driver.enable_torque(sid, enable)
            time.sleep(0.005) # 稍微延时防止指令丢失

    def get_raw_positions(self):
        """读取所有舵机的原始数值 (0-4095)"""
        positions = {}
        for sid in self.servo_ids:
            pos = self.driver.get_position(sid)
            # 如果读不到(-1)，这里暂时保留上一帧数据或者返回None，视策略而定
            # 这里简单处理：如果读不到，就设为 -1
            positions[sid] = pos
        return positions

    def get_angles(self):
        """
        读取并计算当前角度 (单位：度)
        Angle = (Current - Home) * Ratio * Direction
        """
        raw_data = self.get_raw_positions()
        angles = {}
        
        for sid, raw_val in raw_data.items():
            if raw_val == -1:
                angles[sid] = None # 传感器故障
                continue
            
            # 1. 计算相对偏差
            offset = self.home_offsets.get(sid, 2048)
            delta = raw_val - offset
            
            # 2. 转换为角度
            # STS3215: 4096 step = 360 degree => 1 step = 0.08789 degree
            deg = delta * (STS_DEGREE_RANGE / STS_RESOLUTION)
            
            # 3. 应用方向修正
            direction = self.directions.get(sid, 1)
            final_angle = deg * direction
            
            angles[sid] = round(final_angle, 2)
            
        return angles

    # ================= 校准功能 =================
    
    def calibrate_home(self):
        """
        【校准程序】
        读取当前所有关节的位置，并将其保存为 '零点' (0度状态)
        """
        print("\n=== Calibrating Home Position ===")
        print("Reading current positions...")
        
        current_pos = self.get_raw_positions()
        
        # 检查是否有读取错误的舵机
        if -1 in current_pos.values():
            print("❌ Error: Some servos are not responding. Check connection.")
            print(f"Readings: {current_pos}")
            return
            
        self.home_offsets = current_pos
        self.save_config()
        print("✅ Calibration Saved! This pose is now 0.0 degrees.")
        print(f"Offsets: {self.home_offsets}")

    def set_direction(self, servo_id, direction):
        """设置某关节是否反向 (1 或 -1)"""
        if direction not in [1, -1]:
            print("Direction must be 1 or -1")
            return
        self.directions[servo_id] = direction
        self.save_config()

    def save_config(self):
        config = {
            "home_offsets": self.home_offsets,
            "directions": self.directions
        }
        with open(DEFAULT_CONFIG_FILE, 'w') as f:
            json.dump(config, f, indent=4)
        print(f"Config saved to {DEFAULT_CONFIG_FILE}")

    def load_config(self):
        if os.path.exists(DEFAULT_CONFIG_FILE):
            try:
                with open(DEFAULT_CONFIG_FILE, 'r') as f:
                    config = json.load(f)
                    # 转换 key 从 str 回 int (json key 只能是 str)
                    self.home_offsets = {int(k): v for k, v in config.get("home_offsets", {}).items()}
                    self.directions = {int(k): v for k, v in config.get("directions", {}).items()}
                print("Configuration loaded.")
            except Exception as e:
                print(f"Failed to load config: {e}")
        else:
            print("No config file found. Using defaults.")

# ================= 调试与校准入口 =================
if __name__ == "__main__":
    # 配置你的端口
    PORT = '/dev/leader_arm_right' 
    
    leader = LeaderArm(PORT)
    
    print("\n-------------------------------------------")
    print("  Leader Arm Utility")
    print("-------------------------------------------")
    print("1. Set Torque OFF (Free Move mode)")
    print("2. Set Torque ON  (Hold mode)")
    print("3. Calibrate HOME (Set current pose as 0)")
    print("4. Monitor Angles (Real-time)")
    print("q. Quit")
    
    try:
        while True:
            cmd = input("\nEnter command: ")
            
            if cmd == '1':
                leader.set_torque(False)
                print("Arm is now FREE.")
                
            elif cmd == '2':
                leader.set_torque(True)
                print("Arm is now LOCKED.")
                
            elif cmd == '3':
                confirm = input("Are you sure? Ensure arm is in Zero Pose. (y/n): ")
                if confirm.lower() == 'y':
                    leader.calibrate_home()
                    
            elif cmd == '4':
                print("Monitoring... Press Ctrl+C to stop.")
                try:
                    while True:
                        angles = leader.get_angles()
                        # 格式化输出
                        out = " | ".join([f"J{k}:{v:>6.1f}°" for k, v in angles.items()])
                        print(f"\r{out}", end="", flush=True)
                        time.sleep(0.05)
                except KeyboardInterrupt:
                    print("\nStopped monitoring.")
                    
            elif cmd == 'q':
                break
                
    except KeyboardInterrupt:
        pass
    finally:
        leader.driver.close()