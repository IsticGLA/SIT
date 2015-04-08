sudo apt-get update

sudo apt-get install -y python3
sudo apt-get install -y python3-pip
sudo apt-get install python3-dev
sudo pip3 install flask-restful

cd
mkdir install
cd install
wget http://pyyaml.org/download/pyyaml/PyYAML-3.10.tar.gz
tar xvf PyYAML-3.10.tar.gz
cd PyYAML-3.10
sudo python3 setup.py install
cd ..
wget http://python-distribute.org/distribute_setup.py
sudo python3 distribute_setup.py
cd ..
git clone git://github.com/ros/rospkg.git
cd rospkg
sudo python3 setup.py install
cd ..
git clone git://github.com/ros-infrastructure/catkin_pkg.git
cd catkin_pkg
sudo python3 setup.py install
cd ..
git clone git://github.com/ros/catkin.git
cd catkin
sudo python3 setup.py install
echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
echo "export ROS_IP=192.168.33.10" >> ~/.bashrc
source ~/.bashrc

