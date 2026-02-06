import serial.tools.list_ports

def scan_ports():
    print("Scanning USB Ports for Location ID...\n")
    print(f"{'DEVICE':<15} {'SERIAL':<15} {'LOCATION':<15} {'DESCRIPTION'}")
    print("-" * 80)
    
    ports = serial.tools.list_ports.comports()
    for p in ports:
        # p.location 就是我们需要的物理地址 (例如 1-2.4)
        loc = p.location if p.location else "None"
        sn = p.serial_number if p.serial_number else "None"
        print(f"{p.device:<15} {sn:<15} {loc:<15} {p.description}")

    print("-" * 80)
    print("Tips: 'LOCATION' is the physical USB port ID.")
    print("      Use this string in your config to distinguish identical devices.")

if __name__ == "__main__":
    scan_ports()