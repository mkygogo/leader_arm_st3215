import time
import numpy as np
import threading
import sys
import signal

# 导入你的两个驱动
from leader_arm_st3215 import LeaderArm
from mk_driver import MKRobotStandalone

# ================= 配置区域 =================

# 1. 端口配置 (建议使用 udev 固定名字，或者填 ttyACM0/1)
LEADER_PORT = '/dev/leader_arm_right'   
FOLLOWER_PORT = '/dev/ttyACM1'

# 【关键设置】主手夹爪的物理角度范围
# 请根据你刚才测量的数值修改这里！
# 逻辑：当主手角度 = OPEN_DEG 时，从手完全张开
#       当主手角度 = CLOSE_DEG 时，从手完全闭合
LEADER_GRIPPER_OPEN_DEG = 50.0    # 你的主手张开时的角度
LEADER_GRIPPER_CLOSE_DEG = 0.0  # 你的主手捏紧时的角度 (如果方向反了，这里数值可以比Open小)

# 关节方向修正 [J1, J2, J3, J4, J5, J6, Gripper]
# 1.0 为同向，-1.0 为反向
MAPPING_DIRECTION = np.array([1.0, 1.0, 1.0, 1.0, -1.0, 1.0, 1.0])

# ====================================================

class TeleopSystem:
    def __init__(self):
        self.running = True
        
        print("[System] Initializing Leader Arm (STS3215)...")
        try:
            self.leader = LeaderArm(LEADER_PORT)
            self.leader.set_torque(False) 
        except Exception as e:
            print(f"❌ Failed to connect Leader: {e}")
            sys.exit(1)

        print("[System] Initializing Follower Arm (MKRobot)...")
        try:
            # joint_velocity_scaling 建议 0.3 - 0.5，夹爪需要反应快一点
            self.follower = MKRobotStandalone(port=FOLLOWER_PORT, joint_velocity_scaling=0.1)
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
        # 1. 计算当前角度在 [Open, Close] 区间内的百分比
        # 即使 Open > Close (反向安装)，这个公式依然成立
        span = LEADER_GRIPPER_CLOSE_DEG - LEADER_GRIPPER_OPEN_DEG
        if abs(span) < 0.1: return 0.0 # 防止除零
        
        ratio = (raw_deg - LEADER_GRIPPER_OPEN_DEG) / span
        
        # 2. 限制在 0.0 到 1.0 之间 (这就是死区过滤)
        ratio = np.clip(ratio, 0.0, 1.0)
        
        return ratio

    def run(self):
        print("\n==================================================")
        print("   MKRobot Teleoperation System (Continuous Gripper)")
        print("==================================================")
        print(f"Gripper Config: Open={LEADER_GRIPPER_OPEN_DEG}°, Close={LEADER_GRIPPER_CLOSE_DEG}°")
        print("Step 1. Move Leader to Match Follower.")
        print("Step 2. Press [ENTER] to Start.")
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
                gripper_val = self.map_gripper(gripper_raw) # 结果是 0.0 ~ 1.0
                target_state.append(gripper_val)

                # 转 Numpy 并应用方向修正
                action_array = np.array(target_state, dtype=np.float32)
                final_action = action_array * MAPPING_DIRECTION

                # --- 4. 发送指令 ---
                self.follower.send_action(final_action)

                # --- 5. 调试打印 (只打印夹爪信息，方便你观察) ---
                # Raw: 主手实际角度 -> Out: 发送给从手的值
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