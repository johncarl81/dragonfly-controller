

sudo apt update
sudo apt upgrade -y
sudo apt install -y openssh-server
sudo apt install -y python-dev python-opencv python-wxgtk3.0 python-pip python-matplotlib python-pygame python-lxml python-yaml
sudo apt remove ModemManager
sudo apt install -y ros-kinetic-mavros ros-kinetic-mavros-extras
sudo apt install -y python-rosinstall python-rosinstall-generator python-wstool build-essential
sudo apt autoclean
sudo apt autoremove

