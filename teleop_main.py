import time
import numpy as np
import sys
import serial.tools.list_ports

# 导入你的两个驱动
from leader_arm_st3215 import LeaderArm
from mk_driver import MKRobotStandalone

# ================= 配置区域 (混合匹配模式) =================

# 你可以在这里填入：
# 1. 唯一的序列号 (推荐用于 Leader)
# 2. 物理 Location 字符串 (推荐用于 Follower，当 Serial 重复时)

DEVICE_ID_CFG = {
    # 请替换为你实际单臂使用的 Serial 或 Location
    'leader':    '5A68009611',   # 示例: 右主手的序列号
    'follower':  '1-7:1.0'       # 示例: 右从手的 Location
}

# 【关键设置】主手夹爪的物理角度范围
LEADER_GRIPPER_OPEN_DEG = 50.0   # 张开时的角度
LEADER_GRIPPER_CLOSE_DEG = 0.0   # 捏紧时的角度

# 关节方向修正 [J1, J2, J3, J4, J5, J6, Gripper]
# 1.0 为同向，-1.0 为反向
MAPPING_DIRECTION = np.array([1.0, 1.0, 1.0, 1.0, -1.0, 1.0, 1.0])

# ====================================================================

class AutoPortFinder:
    @staticmethod
    def find_ports(target_map):
        print("\n>> Scanning hardware ports...")
        found_ports = {}
        ports = list(serial.tools.list_ports.comports())
        all_success = True
        
        for name, target_id in target_map.items():
            matched_dev = None
            for p in ports:
                p_sn = p.serial_number if p.serial_number else ""
                p_loc = p.location if p.location else ""
                
                if target_id == p_sn:
                    matched_dev = p.device
                    print(f"   ✅ [{name}] Matched Serial: {target_id} -> {matched_dev}")
                    break
                
                if target_id == p_loc:
                    matched_dev = p.device
                    print(f"   ✅ [{name}] Matched Location: {target_id} -> {matched_dev}")
                    break
            
            if matched_dev:
                found_ports[name] = matched_dev
            else:
                print(f"   ❌ [{name}] Device '{target_id}' NOT found!")
                found_ports[name] = None
                all_success = False
                
        return found_ports, all_success


class TeleopSystem:
    def __init__(self):
        self.running = True
        
        print("\n=== Initializing Single Arm Teleoperation (Location/Serial) ===")
        
        # 1. 智能查找端口
        resolved_ports, success = AutoPortFinder.find_ports(DEVICE_ID_CFG)
        
        if not success or not resolved_ports.get('leader') or not resolved_ports.get('follower'):
            print("\n❌ CRITICAL: Missing essential devices. Please check USB connections.")
            sys.exit(1)

        leader_port = resolved_ports['leader']
        follower_port = resolved_ports['follower']

        # 2. 初始化 Leader
        print(f"\n[System] Initializing Leader Arm on {leader_port}...")
        try:
            # 兼容修改过后的 LeaderArm (带 config_file 参数)
            self.leader = LeaderArm(leader_port, config_file="leader_single_config.json")
            self.leader.set_torque(False) 
        except Exception as e:
            print(f"❌ Failed to connect Leader: {e}")
            sys.exit(1)

        # 3. 初始化 Follower
        print(f"[System] Initializing Follower Arm on {follower_port}...")
        try:
            # joint_velocity_scaling 建议 0.1 - 0.3，保证平滑
            self.follower = MKRobotStandalone(port=follower_port, joint_velocity_scaling=0.1)
            self.follower.connect()
        except Exception as e:
            print(f"❌ Failed to connect Follower: {e}")
            self.leader.driver.close()
            sys.exit(1)

    def deg_to_rad(self, deg):
        return deg * (np.pi / 180.0)

    def map_gripper(self, raw_deg):
        """
        连续线性映射：
        将主手角度 (raw_deg) 映射为从手比例 (0.0 ~ 1.0)
        """
        span = LEADER_GRIPPER_CLOSE_DEG - LEADER_GRIPPER_OPEN_DEG
        if abs(span) < 0.1: return 0.0 # 防止除零
        
        ratio = (raw_deg - LEADER_GRIPPER_OPEN_DEG) / span
        ratio = np.clip(ratio, 0.0, 1.0)
        return ratio

    def run(self):
        print("\n==================================================")
        print("   MKRobot Teleoperation System (Auto Port)")
        print("==================================================")
        print(f"Gripper Config: Open={LEADER_GRIPPER_OPEN_DEG}°, Close={LEADER_GRIPPER_CLOSE_DEG}°")
        print("Step 1. Move Leader to Match Follower.")
        print("Step 2. Press [ENTER] to Calibrate & Start.")
        input(">> Press Enter... ")

        self.leader.calibrate_home()
        print("✅ Leader Calibrated. GO!")

        try:
            while self.running:
                loop_start = time.time()

                # --- 1. 获取主手数据 ---
                leader_angles = self.leader.get_angles()
                if None in leader_angles.values(): continue

                # --- 2. 映射关节 J1-J6 (角度 -> 弧度) ---
                target_state = []
                for i in range(1, 7):
                    deg = leader_angles[i]
                    rad = self.deg_to_rad(deg)
                    target_state.append(rad)
                
                # --- 3. 映射夹爪 (连续映射) ---
                gripper_raw = leader_angles[7]
                gripper_val = self.map_gripper(gripper_raw) 
                target_state.append(gripper_val)

                # 转 Numpy 并应用方向修正
                action_array = np.array(target_state, dtype=np.float32)
                final_action = action_array * MAPPING_DIRECTION

                # --- 4. 发送指令 ---
                self.follower.send_action(final_action)

                # --- 5. 调试打印 ---
                print(f"\r[Gripper] Raw: {gripper_raw:5.1f}° -> Out: {gripper_val:4.2f} | J1: {final_action[0]:5.2f}", end="")

                # 50Hz Loop
                elapsed = time.time() - loop_start
                if elapsed < 0.02:
                    time.sleep(0.02 - elapsed)

        except KeyboardInterrupt:
            print("\nStopping...")
        finally:
            self.cleanup()

    def cleanup(self):
        self.leader.driver.close()
        self.follower.close()

if __name__ == "__main__":
    sys = TeleopSystem()
    sys.run()