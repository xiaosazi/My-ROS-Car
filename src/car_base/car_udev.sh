echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE:="0777", SYMLINK+="mycar"' >/etc/udev/rules.d/my_car.rules

service udev reload
sleep 2
service udev restart


