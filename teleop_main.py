import time
import numpy as np
import sys
import argparse
import serial.tools.list_ports

# 导入驱动
from leader_arm_st3215 import LeaderArm
from mk_driver import MKRobotStandalone

# ================= 左右臂配置字典 =================
# 在这里统一定义左右两边的所有差异化参数

ARM_CONFIGS = {
    'right': {
        'leader_id':    '5A68009611',     # 右主手 Serial 或 Location
        'follower_id':  '1-7:1.0',        # 右从手 Location
        'gripper_open': 50.0,
        'gripper_close': 0.0,
        'direction':    np.array([1.0, 1.0, 1.0, 1.0, -1.0, 1.0, 1.0]),
        'config_file':  'leader_right_config.json'
    },
    'left': {
        'leader_id':    '5A68012049',     # 左主手 Serial 或 Location
        'follower_id':  '1-9.1:1.0',      # 左从手 Location
        'gripper_open': 50.0,
        'gripper_close': 0.0,
        'direction':    np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]),
        'config_file':  'leader_left_config.json'
    }
}
# ==================================================

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
    def __init__(self, side):
        self.running = True
        self.side = side
        self.cfg = ARM_CONFIGS[side]
        
        print(f"\n=== Initializing Single Arm Teleoperation ({side.upper()} ARM) ===")
        
        # 1. 设置要查找的端口目标
        target_device_map = {
            'leader': self.cfg['leader_id'],
            'follower': self.cfg['follower_id']
        }
        
        # 2. 智能查找端口
        resolved_ports, success = AutoPortFinder.find_ports(target_device_map)
        
        if not success or not resolved_ports.get('leader') or not resolved_ports.get('follower'):
            print("\n❌ CRITICAL: Missing essential devices. Please check USB connections.")
            sys.exit(1)

        leader_port = resolved_ports['leader']
        follower_port = resolved_ports['follower']

        # 3. 初始化 Leader
        print(f"\n[System] Initializing {side.upper()} Leader Arm on {leader_port}...")
        try:
            self.leader = LeaderArm(leader_port, config_file=self.cfg['config_file'])
            self.leader.set_torque(False) 
        except Exception as e:
            print(f"❌ Failed to connect Leader: {e}")
            sys.exit(1)

        # 4. 初始化 Follower
        print(f"[System] Initializing {side.upper()} Follower Arm on {follower_port}...")
        try:
            self.follower = MKRobotStandalone(port=follower_port, joint_velocity_scaling=0.1)
            self.follower.connect()
        except Exception as e:
            print(f"❌ Failed to connect Follower: {e}")
            self.leader.driver.close()
            sys.exit(1)

    def deg_to_rad(self, deg):
        return deg * (np.pi / 180.0)

    def map_gripper(self, raw_deg):
        """连续线性映射"""
        open_deg = self.cfg['gripper_open']
        close_deg = self.cfg['gripper_close']
        
        span = close_deg - open_deg
        if abs(span) < 0.1: return 0.0 
        
        ratio = (raw_deg - open_deg) / span
        ratio = np.clip(ratio, 0.0, 1.0)
        return ratio

    def run(self):
        print("\n==================================================")
        print(f"   MKRobot Teleop: {self.side.upper()} ARM Active")
        print("==================================================")
        print(f"Gripper Config: Open={self.cfg['gripper_open']}°, Close={self.cfg['gripper_close']}°")
        print("Step 1. Move Leader to Match Follower.")
        print("Step 2. Press [ENTER] to Calibrate & Start.")
        input(">> Press Enter... ")

        self.leader.calibrate_home()
        print(f"✅ {self.side.upper()} Leader Calibrated. GO!")

        try:
            while self.running:
                loop_start = time.time()

                # --- 1. 获取主手数据 ---
                leader_angles = self.leader.get_angles()
                if None in leader_angles.values(): continue

                # --- 2. 映射关节 J1-J6 ---
                target_state = []
                for i in range(1, 7):
                    deg = leader_angles[i]
                    rad = self.deg_to_rad(deg)
                    target_state.append(rad)
                
                # --- 3. 映射夹爪 ---
                gripper_raw = leader_angles[7]
                gripper_val = self.map_gripper(gripper_raw) 
                target_state.append(gripper_val)

                # --- 4. 应用专属方向修正 ---
                action_array = np.array(target_state, dtype=np.float32)
                final_action = action_array * self.cfg['direction']

                # --- 5. 发送指令 ---
                self.follower.send_action(final_action)

                # --- 6. 调试打印 ---
                print(f"\r[{self.side.upper()} Gripper] Raw: {gripper_raw:5.1f}° -> Out: {gripper_val:4.2f} | J1: {final_action[0]:5.2f}", end="")

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
    # 使用 argparse 解析命令行参数
    parser = argparse.ArgumentParser(description="Single Arm Teleoperation Script")
    
    # 添加 --side 参数，默认值为 'right'，只允许输入 'left' 或 'right'
    parser.add_argument(
        '--side', 
        type=str, 
        default='right', 
        choices=['left', 'right'],
        help="Specify which arm to control: 'left' or 'right' (default is 'right')"
    )
    
    args = parser.parse_args()
    
    # 将解析到的 side 参数传入系统
    sys_app = TeleopSystem(side=args.side)
    sys_app.run()