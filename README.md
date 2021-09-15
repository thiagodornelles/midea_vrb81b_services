# midea_vrb81b_services

On /boot/config.txt

Remove any other i2c config and put

dtparam=i2c_arm=on,i2c_arm_baudrate=400000


Copy midea.lircd.conf to /etc/lirc/lircd.conf.d/

