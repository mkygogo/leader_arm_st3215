tool_auto_set_id.py用来设置舵机编号，会先读出老的编号，等待输入新编号。
tool_set_id.py用来设置舵机编号，要先输入老的id，建议直接用上面那个。
read_7_axis.py 可以直接把整个机械臂舵机实时数据读出来。
teleop_main.py 用来摇操机械臂。

现在代码里根据串口信息来判断左右，代码里根据串口信息写死左右臂。
两个leader臂根据tool_list_usb_serial_pots.py获取serial id。
两个follower臂的serial id是一样的，就通过tool_list_usb_location.py的方法获取location信息。


########################这个方法不太好，总是要修改电脑配置，可移植性差#######################################
leader臂通过udev把ttyACM*映射成了leader_arm_right, leader_arm_left。这个电脑上必须手动配置，否则会报错。
ttyACM映射方法：
    1:使用 udevadm 命令查看它的详细信息，特别是序列号（SERIAL）：udevadm info -a -n /dev/ttyACM0 | grep "serial"
    ATTRS{serial}=="A1012345" 
    2:编写 udev 规则: sudo vim /etc/udev/rules.d/99-robotic-arm.rules
        SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{serial}=="1234567A", SYMLINK+="leader_arm_right", MODE="0666"
    3:生效与测试
        重新加载规则:sudo udevadm control --reload-rules
                    sudo udevadm trigger
    
        查看是否成功：ls -l /dev/leader_arm_right
