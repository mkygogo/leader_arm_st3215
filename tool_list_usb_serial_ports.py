import serial.tools.list_ports

def list_ports():
    print("Scanning USB Serial Devices...\n")
    ports = serial.tools.list_ports.comports()
    
    if not ports:
        print("❌ No serial devices found.")
        return

    print(f"{'DEVICE':<20} {'SERIAL NUMBER':<20} {'MANUFACTURER'}")
    print("-" * 60)
    
    for p in ports:
        # 有些设备序列号可能是 None，做个处理
        sn = p.serial_number if p.serial_number else "None"
        manu = p.manufacturer if p.manufacturer else "Unknown"
        print(f"{p.device:<20} {sn:<20} {manu}")

    print("-" * 60)
    print("\n[Tips] If multiple devices have 'None' or identical serial numbers,")
    print("       this method will not work (hardware limitation).")

if __name__ == "__main__":
    list_ports()