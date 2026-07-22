#CP2102 串口号0003 设置别名为yesense_IMU
echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60",ATTRS{serial}=="0003", MODE:="0777", GROUP:="dialout", SYMLINK+="yesense_IMU"' >/etc/udev/rules.d/yesense_imu.rules

#CH9102 ，如果已经安装了驱动使用CH343	
echo 'KERNEL=="ttyCH343USB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4", ATTRS{serial}=="0003", MODE:="0777", GROUP:="dialout", SYMLINK+="yesense_IMU"' >/etc/udev/rules.d/yesense_imu_343.rules

#CH9102  如果没有安装驱动
#udevadm info --name=/dev/ttyACM0 --attribute-walk | grep serial
# echo  'KERNEL=="ttyACM*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4", ATTRS{serial}=="0003", MODE:="0777", GROUP:="dialout", SYMLINK+="yesense_IMU"' >/etc/udev/rules.d/yesense_imu_ACM.rules

echo  'KERNEL=="ttyACM*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4",  MODE:="0777", GROUP:="dialout", SYMLINK+="yesense_IMU"' >/etc/udev/rules.d/yesense_imu_ACM.rules

service udev reload
sleep 2
service udev restart
