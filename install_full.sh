#!/bin/sh

echo "eFinder CM install on Pi CM4"
echo " "

echo "Nexus usb via CP230x uart and Aux usb host port"
sudo tee -a /boot/firmware/config.txt > /dev/null <<EOT
arm_freq=600
dtoverlay=imx477,cam0
dtoverlay=dwc2,dr_mode=host
dtoverlay=uart3
enable_uart=1
dtoverlay=disable-bt
dtoverlay=i2c-gpio,i2c_gpio_sda=16,i2c_gpio_scl=20,bus=3
dtparam=act_led_trigger=none
dtparam=pwr_led_trigger=none
EOT

echo "*****************************************************************************"
echo "Updating Pi OS & packages"
echo "*****************************************************************************"
sudo apt update
#sudo apt upgrade -y
echo " "
echo "*****************************************************************************"
echo "Installing additional Debian and Python packages"
echo "*****************************************************************************"
sudo apt install -m -y python3-pip
sudo apt install -y python3-serial
sudo apt install -y python3-psutil
sudo apt install -y python3-pil
sudo apt install -y python3-pil.imagetk
sudo apt install -y git
sudo apt install -y python3-smbus
sudo apt install -y python3-picamera2
sudo apt install -y python3-scipy

HOME=/home/efinder
cd $HOME
echo " "

python -m venv /home/efinder/venv-efinder --system-site-packages
venv-efinder/bin/python venv-efinder/bin/pip install adafruit_extended_bus
venv-efinder/bin/python venv-efinder/bin/pip install adafruit-circuitpython-bno08x

cd $HOME
echo " "
echo "*****************************************************************************"
echo "Downloading eFinder_CM from AstroKeith GitHub"
echo "*****************************************************************************"
sudo -u efinder git clone https://github.com/AstroKeith/eFinder_CM.git
echo " "

cd $HOME
echo " "
echo "*****************************************************************************"
echo "Unpacking eFinder_CM & configuring"
echo "*****************************************************************************"
echo "tmpfs /var/tmp tmpfs nodev,nosuid,size=10M 0 0" | sudo tee -a /etc/fstab > /dev/null
mkdir /home/efinder/Solver
mkdir /home/efinder/Solver/images
mkdir /home/efinder/uploads
sudo chmod a+rwx /home/efinder/uploads

cp /home/efinder/eFinder_CM/Solver/*.* /home/efinder/Solver
echo "tmpfs /home/efinder/Solver/images tmpfs nodev,nosuid,size=10M 0 0" | sudo tee -a /etc/fstab > /dev/null

cd $HOME
echo " "
echo "*****************************************************************************"
echo "Installing Samba file share support"
echo "*****************************************************************************"
sudo apt install -y samba samba-common-bin
sudo tee -a /etc/samba/smb.conf > /dev/null <<EOT
[efindershare]
path = /home/efinder
writeable=Yes
create mask=0777
directory mask=0777
public=no
EOT
username="efinder"
pass="efinder"
(echo $pass; sleep 1; echo $pass) | sudo smbpasswd -a -s $username
sudo systemctl restart smbd

cd $HOME
echo " "
echo "*****************************************************************************"
echo "installing tetra and olive-solve"
echo "*****************************************************************************"

venv-efinder/bin/python venv-efinder/bin/pip install Solver/tetra3-0.1.0-cp311-cp311-manylinux_2_35_aarch64.whl
venv-efinder/bin/python venv-efinder/bin/pip install Solver/olive_solve-0.1.0-cp38-abi3-manylinux_2_34_aarch64.whl

echo " "
echo "*****************************************************************************"
echo "Setting up web page server"
echo "*****************************************************************************"
sudo apt-get install -y apache2
sudo apt-get install -y php8.2
sudo chmod a+rwx /home/efinder
sudo chmod a+rwx /home/efinder/Solver/images
sudo cp /home/efinder/eFinder_CM/Solver/index.php /var/www/html
sudo cp /home/efinder/eFinder_CM/Solver/upload.php /var/www/html
sudo cp /home/efinder/eFinder_CM/Solver/log.php /var/www/html
sudo cp /home/efinder/eFinder_CM/Solver/updater.html /var/www/html
sudo cp /home/efinder/eFinder_CM/Solver/user.ini /etc/php/8.2/apache2/conf.d
sudo cp /home/efinder/eFinder_CM/Solver/user.ini /etc/php/8.2/cli/conf.d
sudo mv /var/www/html/index.html /var/www/html/apacheindex.html
sudo chmod -R a+rwx /var/www/html

cd $HOME
echo " "
echo "*****************************************************************************"
echo "Final eFinder_CM configuration setting"
echo "*****************************************************************************"



sudo python /home/efinder/Solver/cmdlineUpdater.py

sudo chmod a+rwx eFinder_CM/Solver/my_cron
sudo cp /home/efinder/eFinder_CM/Solver/my_cron /etc/cron.d

echo 'vm.swappiness = 0' | sudo tee -a /etc/sysctl.conf > /dev/null
sudo raspi-config nonint do_boot_behaviour B2
sudo raspi-config nonint do_ssh 0
sudo raspi-config nonint do_i2c 0
sudo raspi-config nonint do_serial_cons 1

sudo python /home/efinder/Solver/configUpdater.py
sudo cp newconfig.txt /boot/firmware/config.txt

cd $HOME

#sudo reboot now

