#CP2102 串口号0003 设置别名为FDI_IMU_GNSS
echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0777", SYMLINK+="FDI_IMU_GNSS"' >/etc/udev/rules.d/fdi_imu_gnss.rules

#CH9102，同时系统安装了对应驱动 串口号0003 设置别名为FDI_IMU_GNSS
# echo  'KERNEL=="ttyCH343USB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4", MODE:="0777",  SYMLINK+="FDI_IMU_GNSS"' >/etc/udev/rules.d/fdi_imu_gnss2.rules

#CH9102，同时系统没有安装对应驱动 串口号0003 设置别名为FDI_IMU_GNSS
#echo  'KERNEL=="ttyACM*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4", MODE:="0777",  SYMLINK+="FDI_IMU_GNSS"' >/etc/udev/rules.d/fdi_imu_gnss3.rules

#CH340，直接设置别名为FDI_IMU_GNSS
#echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE:="0777",  SYMLINK+="FDI_IMU_GNSS"' >/etc/udev/rules.d/fdcontroller_340.rules

service udev reload
sleep 2
service udev restart


