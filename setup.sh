
sudo cp system_config/10-dragonfly.rules /etc/udev/rules.d/
sudo cp system_config/rc.local /etc/rc.local
sudo cp system_config/interfaces /etc/network/interfaces

sudo apt update
sudo apt upgrade -y
sudo apt install -y openssh-server
sudo apt install -y python-dev python-opencv python-wxgtk3.0 python-pip python-matplotlib python-pygame python-lxml python-yaml
sudo pip install MAVProxy
echo "export PATH=$PATH:$HOME/.local/bin" >> ~/.bashrc
sudo adduser odroid dialout
sudo apt remove ModemManager
sudo ./datasets/install_geographiclib_datasets.sh
sudo apt install -y ros-kinetic-mavros ros-kinetic-mavros-extras
sudo apt install -y python-rosinstall python-rosinstall-generator python-wstool build-essential
sudo apt autoclean
sudo apt autoremove

