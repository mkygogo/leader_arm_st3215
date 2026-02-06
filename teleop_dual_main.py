import time
import numpy as np
import sys
import serial.tools.list_ports

# 导入驱动
from leader_arm_st3215 import LeaderArm
from mk_driver import MKRobotStandalone

# ================= 配置区域 (混合匹配模式) =================

# 你可以在这里填入：
# 1. 唯一的序列号 (推荐用于 Leader)
# 2. 物理 Location 字符串 (推荐用于 Follower，当你查到 serial 一样时)

DEVICE_ID_CFG = {
    # 主手：如果有唯一序列号，就填序列号
    'left_leader':    '5A68012049',      
    'right_leader':   '5A68009611',      
    
    # 从手：序列号重复，填你在第一步查到的 LOCATION (例如 '1-1.3')
    'left_follower':  '1-9.1:1.0',  
    'right_follower': '1-7:1.0'   
}

# --- 夹爪配置 ---
GRIPPER_CFG = {
    'left':  {'open': 50.0, 'close': 0.0}, 
    'right': {'open': 50.0, 'close': 0.0}
}

# --- 关节方向 ---
DIR_CFG = {
    'left':  np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]),
    'right': np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
}

# ====================================================================

class AutoPortFinder:
    @staticmethod
    def find_ports(target_map):
        print("\n>> Scanning hardware ports...")
        found_ports = {}
        
        # 获取当前所有串口的完整信息
        ports = list(serial.tools.list_ports.comports())
        
        # 为了调试，先打印一下当前扫描到的信息
        # print("   [Debug] Current visible devices:")
        # for p in ports:
        #     print(f"   - {p.device} | SN: {p.serial_number} | Loc: {p.location}")

        all_success = True
        
        for name, target_id in target_map.items():
            matched_dev = None
            
            # 遍历所有物理端口寻找匹配
            for p in ports:
                # 核心逻辑：既比对序列号，也比对物理位置 (Location)
                # 只要有一个对上了，就认为是它
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

class DualTeleopSystem:
    def __init__(self):
        self.running = True
        self.arms = {}
        
        print("\n=== Initializing Dual Arm Teleoperation (Location/Serial) ===")
        
        # 1. 智能查找端口
        resolved_ports, success = AutoPortFinder.find_ports(DEVICE_ID_CFG)
        
        if not success:
            print("\n⚠️  Some devices missing. Checking what can be started...")

        # 2. 初始化左臂
        if resolved_ports.get('left_leader') and resolved_ports.get('left_follower'):
            print("\n>> Setup LEFT Arm System...")
            self.arms['left'] = self.setup_arm_pair(
                'left', 
                resolved_ports['left_leader'], 
                resolved_ports['left_follower'],
                "leader_left_config.json"
            )
        else:
            self.arms['left'] = None
            print(">> Skipped LEFT arm.")

        # 3. 初始化右臂
        if resolved_ports.get('right_leader') and resolved_ports.get('right_follower'):
            print("\n>> Setup RIGHT Arm System...")
            self.arms['right'] = self.setup_arm_pair(
                'right', 
                resolved_ports['right_leader'], 
                resolved_ports['right_follower'],
                "leader_right_config.json"
            )
        else:
            self.arms['right'] = None
            print(">> Skipped RIGHT arm.")

        if not self.arms['left'] and not self.arms['right']:
            print("❌ No active arms. Exiting.")
            sys.exit(1)

    def setup_arm_pair(self, side, leader_port, follower_port, config_file):
        pair = {'leader': None, 'follower': None}
        try:
            pair['leader'] = LeaderArm(leader_port, config_file=config_file)
            pair['leader'].set_torque(False)
        except Exception as e:
            print(f"  ⚠️ Connect {side} Leader Failed: {e}")
            return None

        try:
            pair['follower'] = MKRobotStandalone(port=follower_port, joint_velocity_scaling=0.15)
            pair['follower'].connect()
        except Exception as e:
            print(f"  ⚠️ Connect {side} Follower Failed: {e}")
            if pair['leader']: pair['leader'].driver.close()
            return None
            
        print(f"  ✅ {side.upper()} System Ready.")
        return pair

    def deg_to_rad(self, deg):
        return deg * (np.pi / 180.0)

    def map_gripper(self, raw_deg, side):
        cfg = GRIPPER_CFG[side]
        span = cfg['close'] - cfg['open']
        if abs(span) < 0.1: return 0.0
        ratio = (raw_deg - cfg['open']) / span
        return np.clip(ratio, 0.0, 1.0)

    def process_single_arm(self, side):
        pair = self.arms.get(side)
        if not pair: return

        leader = pair['leader']
        follower = pair['follower']

        leader_angles = leader.get_angles()
        if None in leader_angles.values(): return

        target_state = []
        for i in range(1, 7):
            target_state.append(self.deg_to_rad(leader_angles[i]))
        
        target_state.append(self.map_gripper(leader_angles[7], side))
        
        action_array = np.array(target_state, dtype=np.float32)
        # 你的 MKRobot 已经处理了物理安装方向，这里只处理镜像逻辑
        follower.send_action(action_array * DIR_CFG[side])
        
        return f"{side[0].upper()}:OK"

    def run(self):
        print("\n==================================================")
        print("   Dual-Arm Teleop Running")
        print("==================================================")
        
        input(">> Press Enter to Calibrate Home Positions... ")
        for side, pair in self.arms.items():
            if pair:
                pair['leader'].calibrate_home()
        print("✅ Calibrated. Loop Starting...")

        try:
            while self.running:
                loop_start = time.time()
                
                self.process_single_arm('left')
                self.process_single_arm('right')
                
                elapsed = time.time() - loop_start
                if elapsed < 0.02:
                    time.sleep(0.02 - elapsed)

        except KeyboardInterrupt:
            print("\nStopping...")
        finally:
            self.cleanup()

    def cleanup(self):
        for side, pair in self.arms.items():
            if pair:
                try:
                    pair['leader'].driver.close()
                    pair['follower'].close()
                except: pass

if __name__ == "__main__":
    sys = DualTeleopSystem()
    sys.run()