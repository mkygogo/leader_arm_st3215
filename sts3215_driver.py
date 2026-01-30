import serial
import time
import struct

class STSServoDriver:
    # 内存表地址 (STS3215 Memory Map)
    # 根据飞特官方手册定义
    SMS_STS_ID = 5
    SMS_STS_BAUD_RATE = 6
    
    SMS_STS_TORQUE_ENABLE = 40
    SMS_STS_ACC = 41
    SMS_STS_GOAL_POSITION = 42
    SMS_STS_GOAL_SPEED = 46
    SMS_STS_LOCK = 55
    
    # 反馈数据地址
    SMS_STS_PRESENT_POSITION = 56
    SMS_STS_PRESENT_SPEED = 58
    SMS_STS_PRESENT_LOAD = 60
    SMS_STS_PRESENT_VOLTAGE = 62
    SMS_STS_PRESENT_TEMPERATURE = 63

    # 指令 (Instruction)
    INST_PING = 1
    INST_READ = 2
    INST_WRITE = 3
    INST_REG_WRITE = 4
    INST_ACTION = 5
    INST_SYNC_WRITE = 131 # 0x83

    def __init__(self, port, baudrate=1000000, timeout=0.05):
        """
        初始化串口
        :param port: 串口号 (Windows: 'COMx', Linux: '/dev/ttyUSBx')
        :param baudrate: 波特率 (默认 1000000)
        :param timeout: 串口读取超时 (秒)
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = None
        
        try:
            self.ser = serial.Serial(port, baudrate, timeout=timeout)
            # 解决Windows下FTDI/CH340延迟过高的问题(可选，视驱动情况而定)
            self.ser.flushInput()
        except Exception as e:
            print(f"[ERROR] Failed to open serial port: {e}")

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()

    def _calc_checksum(self, packet):
        """计算校验和: ~(ID + Length + Instruction + Params) & 0xFF"""
        # packet list: [ID, Length, Instruction, Param1, Param2...]
        # 排除 Header (FF FF)
        total = sum(packet)
        return (~total) & 0xFF

    def _write_packet(self, servo_id, instruction, params=None):
        if params is None:
            params = []
        
        # 构造包: [FF, FF, ID, Length, Instruction, P1...Pn, Checksum]
        length = len(params) + 2 # Length = 指令长度(1) + 参数长度 + 校验和(1)
        
        # 内部处理数据包 (不含Header和Checksum，用于计算)
        payload = [servo_id, length, instruction] + params
        checksum = self._calc_checksum(payload)
        
        # 最终发送的字节流
        full_packet = [0xFF, 0xFF] + payload + [checksum]
        
        if self.ser and self.ser.is_open:
            # 清空输入缓存，防止读到之前的残留数据
            self.ser.flushInput()
            self.ser.write(bytes(full_packet))
            return True
        return False

    def _read_response(self, servo_id, expected_len):
        """
        读取并解析返回包
        STS返回包格式: [FF, FF, ID, Length, Error, Param1...Pn, Checksum]
        """
        if not self.ser or not self.ser.is_open:
            return None

        # 读取 Header (FF FF)
        # 这里做一个简单的状态机寻找包头，防止错位
        header_count = 0
        start_time = time.time()
        
        while True:
            if time.time() - start_time > self.timeout:
                return None # 超时
                
            byte = self.ser.read(1)
            if not byte:
                continue
            
            b = ord(byte)
            if b == 0xFF:
                header_count += 1
            else:
                header_count = 0
            
            if header_count == 2:
                break
        
        # 读剩余部分：ID(1) + Len(1) + Err(1) + Params(n) + Checksum(1)
        # Length 字节代表后面所有字节数 (Err + Params + Checksum)
        # 我们先读 ID 和 Length
        meta = self.ser.read(2)
        if len(meta) < 2:
            return None
            
        resp_id, resp_len = meta[0], meta[1]
        
        # 剩余字节数 = Length - 2 (因为 Length 包含了它自己没包含的 checksum... 不，STS协议Length包含 Error+Params+Chk)
        # STS协议手册：Length = 参数个数 + 2 (Error + Checksum)
        remaining = resp_len - 2 
        if remaining < 0: return None # 异常

        # 读取 Error, Params, Checksum
        # Error(1) + Params(...) + Checksum(1)
        body = self.ser.read(resp_len)
        if len(body) != resp_len:
            return None
            
        error_byte = body[0]
        params = body[1:-1]
        recv_checksum = body[-1]
        
        # 校验ID
        if resp_id != servo_id:
            return None 
            
        # (可选) 校验Checksum，这里略过以提高速度，如需严谨控制可加上
        
        if error_byte != 0:
            print(f"[WARN] Servo {servo_id} Error Byte: {error_byte}")
            
        return params

    # ================= 常用功能封装 =================

    def ping(self, servo_id):
        """检测舵机是否存在"""
        self._write_packet(servo_id, self.INST_PING)
        resp = self._read_response(servo_id, 0)
        return resp is not None

    def enable_torque(self, servo_id, enable=True):
        """开启或关闭力矩 (1=吸合, 0=卸力)"""
        val = 1 if enable else 0
        self._write_packet(servo_id, self.INST_WRITE, [self.SMS_STS_TORQUE_ENABLE, val])

    def set_position(self, servo_id, position, speed=0, acc=0):
        """
        发送位置指令
        :param position: 目标位置 (0-4095) (STS3215 1圈分辨率)
        :param speed: 运动速度 (0-2400) 0为最大速度
        :param acc: 加速度 (0-254) 0为最大加速度
        """
        # 限制范围
        position = max(0, min(4095, position))
        
        # 将数值转换为字节 (Little Endian)
        # struct.pack('<h', val) 会返回 bytes，我们需要 list[int]
        # 写入顺序：Acc(1) + GoalPos(2) + GoalTime/Speed(2)
        # 我们从 ACC 地址(41)开始写，连续写5个字节，或者从 Pos(42) 开始写
        
        # 推荐：使用写地址 42 (Goal Position) 连写4个字节 (Pos + Speed)
        # 或者 简易写法：只写位置
        
        pos_L = position & 0xFF
        pos_H = (position >> 8) & 0xFF
        
        spd_L = speed & 0xFF
        spd_H = (speed >> 8) & 0xFF
        
        # 写入地址 42, 长度4 (Pos_L, Pos_H, Spd_L, Spd_H)
        # Params: [Addr, P1, P2...]
        params = [self.SMS_STS_GOAL_POSITION, pos_L, pos_H, spd_L, spd_H]
        self._write_packet(servo_id, self.INST_WRITE, params)

    def get_position(self, servo_id):
        """
        读取当前位置
        :return: int (0-4095) 或 -1 (读取失败)
        """
        # 读取指令: [Addr, Length]
        # 读取 Current Position (Addr 56), 长度 2
        params = [self.SMS_STS_PRESENT_POSITION, 2]
        self._write_packet(servo_id, self.INST_READ, params)
        
        resp = self._read_response(servo_id, 2)
        if resp and len(resp) == 2:
            # Little Endian 解包
            position = resp[0] + (resp[1] << 8)
            # 处理可能的溢出或符号（STS3215通常是无符号0-4095）
            return position
        return -1

    def get_feedback(self, servo_id):
        """
        一次性读取 位置、速度、负载 (主手模式推荐用这个)
        :return: dict {'pos': int, 'speed': int, 'load': int}
        """
        # 从地址 56 开始读，读 6 个字节 (Pos:2, Spd:2, Load:2)
        params = [self.SMS_STS_PRESENT_POSITION, 6]
        self._write_packet(servo_id, self.INST_READ, params)
        
        resp = self._read_response(servo_id, 6)
        if resp and len(resp) == 6:
            pos = resp[0] + (resp[1] << 8)
            spd = resp[2] + (resp[3] << 8)
            # 速度和负载通常是有符号整数 (int16)
            if spd > 32767: spd -= 65536
            
            load = resp[4] + (resp[5] << 8)
            if load > 32767: load -= 65536
            
            return {'pos': pos, 'speed': spd, 'load': load}
        return None

    def change_id(self, old_id, new_id):
        """修改舵机ID (慎用)"""
        # 1. 解锁 EEPROM (Addr 55 写 0)
        self._write_packet(old_id, self.INST_WRITE, [self.SMS_STS_LOCK, 0])
        time.sleep(0.01)
        # 2. 写入新 ID (Addr 5)
        self._write_packet(old_id, self.INST_WRITE, [self.SMS_STS_ID, new_id])
        time.sleep(0.01)
        # 3. 锁定 EEPROM (Addr 55 写 1)
        self._write_packet(new_id, self.INST_WRITE, [self.SMS_STS_LOCK, 1])
        print(f"Changed ID {old_id} to {new_id}")

# ================= 测试代码 =================
if __name__ == "__main__":
    # 请根据实际情况修改端口
    # Windows: COMx, Mac/Linux: /dev/ttyUSB0
    PORT = '/dev/ttyACM0' 
    
    sts = STSServoDriver(PORT, baudrate=1000000)
    
    try:
        target_id = 1
        print(f"Pinging ID {target_id}...")
        if sts.ping(target_id):
            print("Servo Connected!")
            
            # 读取当前位置
            pos = sts.get_position(target_id)
            print(f"Current Position: {pos}")
            
            # 小幅度移动测试 (危险：请确保电机未连接机械臂或有空间移动)
            # print("Moving to 2048...")
            # sts.enable_torque(target_id, True)
            # sts.set_position(target_id, 2048, speed=1000)
            
            # 循环读取反馈 (主手模式模拟)
            print("Reading feedback loop (Press Ctrl+C to stop)...")
            while True:
                fb = sts.get_feedback(target_id)
                if fb:
                    print(f"ID:{target_id} Pos:{fb['pos']} Spd:{fb['speed']} Load:{fb['load']}")
                time.sleep(0.1)
                
        else:
            print("Servo not found.")
            
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        sts.close()