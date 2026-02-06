import time
import json
import os
from sts3215_driver import STSServoDriver

# 默认配置
DEFAULT_CONFIG_FILE = "leader_config.json"
STS_RESOLUTION = 4096.0
STS_DEGREE_RANGE = 360.0

class LeaderArm:
    # 修改 __init__，增加 config_file 参数
    def __init__(self, port, baudrate=1000000, servo_ids=[1, 2, 3, 4, 5, 6, 7], config_file="leader_config.json"):
        """
        config_file: 用于区分左臂和右臂的校准文件
        """
        self.config_file = config_file  # 保存文件名到实例变量
        self.servo_ids = servo_ids
        self.driver = STSServoDriver(port, baudrate)
        
        # 检查串口
        if not self.driver.ser or not self.driver.ser.is_open:
            raise Exception(f"❌ CRITICAL ERROR: Could not open port {port}.")

        self.home_offsets = {id: 2048 for id in servo_ids}
        self.directions = {id: 1 for id in servo_ids}
        
        self.load_config() # 加载指定的文件

        print(f"Leader Arm Initialized (Config: {self.config_file})")

    # 修改 save_config 使用 self.config_file
    def save_config(self):
        config = {
            "home_offsets": self.home_offsets,
            "directions": self.directions
        }
        with open(self.config_file, 'w') as f:
            json.dump(config, f, indent=4)
        print(f"Config saved to {self.config_file}")

    # 修改 load_config 使用 self.config_file
    def load_config(self):
        if os.path.exists(self.config_file):
            try:
                with open(self.config_file, 'r') as f:
                    config = json.load(f)
                    self.home_offsets = {int(k): v for k, v in config.get("home_offsets", {}).items()}
                    self.directions = {int(k): v for k, v in config.get("directions", {}).items()}
                print(f"Configuration loaded from {self.config_file}.")
            except Exception as e:
                print(f"Failed to load config: {e}")
        else:
            print(f"No config file found ({self.config_file}). Using defaults.")
        
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
        包含【过零点自动修正】逻辑
        """
        raw_data = self.get_raw_positions()
        angles = {}
        
        for sid, raw_val in raw_data.items():
            if raw_val == -1:
                angles[sid] = None
                continue
            
            # 1. 获取零点值
            offset = self.home_offsets.get(sid, 2048)
            
            # 2. 计算原始偏差
            delta = raw_val - offset
            
            # ================= [新增] 过零点处理逻辑 =================
            # STS3215 总分辨率是 4096
            # 如果偏差 > 2048，说明它向负方向跨过了0点，变成了巨大的正数 -> 减去4096
            # 如果偏差 < -2048，说明它向正方向跨过了0点，变成了巨大的负数 -> 加上4096
            
            if delta > 2048:
                delta -= 4096
            elif delta < -2048:
                delta += 4096
            # =======================================================
            
            # 3. 转换为角度 (4096 step = 360 degree)
            deg = delta * (360.0 / 4096.0)
            
            # 4. 应用方向修正
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