# Projet de SIT, Istic 2015 Génie Logiciel en Apprentissage
Système d'Information Tactique pour les pompier avec gestion de flotte de drones.

# Comment initialiser son environnement ?
ROS
-------------------

Le noeud master ROS doit être installé sur une machine permettant une connection didirectionnelle noeud <-> PC hébergeant la simulation morse, et noeud <-> serveur hébergeant nivimoju
Dans notre cas, cette machine est une VM hébergée à l'ISTIC.

configuration à ajouter dans le bashrc de la machine hébergeant ROS et Flask (Ubuntu 14.04):
    export ROS_IP=XX.XX.XX.XX # ip de la machine
    export ROS_MASTER_URI=http://localhost:11311
    
configuration à ajouter dans le bashrc de la machine hébergeant la simulation morse:
    export ROS_IP=$(ip addr | grep 'state UP' -A2 | tail -n1 | awk '{print $2}' | cut -f1 -d'/')
    export ROS_MASTER_URI=http://ip.de.la.vm:11311
    
installation de ROS sur la VM :
--------------------------------

    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list'
    wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -
    sudo apt-get update
    sudo apt-get install xserver-xorg-dev-lts-utopic mesa-common-dev-lts-utopic libxatracker-dev-lts-utopic libopenvg1-mesa-dev-lts-utopic libgles2-mesa-dev-lts-utopic libgles1-mesa-dev-lts-utopic libgl1-mesa-dev-lts-utopic libgbm-dev-lts- utopic libegl1-mesa-dev-lts-utopic
    sudo apt-get install ros-indigo-ros-base
    sudo rosdep init
    rosdep update
    echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    sudo apt-get install python-rosinstall

installation d'opencv et flask sur la VM (python qui commanderas le drone et enverra des images)
--------------------------------------------------------------

    sudo apt-get install libopencv-dev build-essential checkinstall cmake pkg-config yasm libtiff4-dev libjpeg-dev libjasper-dev libavcodec-dev libavformat-dev libswscale-dev libdc1394-22-dev libxine-dev libgstreamer0.10-dev libgstreamer-plugins-base0.10-dev libv4l-dev python3-dev python3-numpy libtbb-dev libqt4-dev libgtk2.0-dev libfaac-dev libmp3lame-dev libopencore-amrnb-dev libopencore-amrwb-dev libtheora-dev libvorbis-dev libxvidcore-dev x264 v4l-utils python3.4-dev python python-pip
    sudo pip install flask
    sudo pip install flask_restful
    git clone https://github.com/Itseez/opencv.git
    cd opencv/
    mkdir release
    cd release/
    cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local -D BUILD_opencv_java=OFF -D WITH_IPP=OFF -D PYTHON_EXECUTABLE=/usr/bin/python3.4 ..
    make
    sudo make install

installation de MORSE sur le PC faisant tourner la simulation
-----------------------------------------------------------------

    sudo apt-get install -y cmake python3 python3-dev blender python3-yaml python3-setuptools build-essential
    cd
    mkdir -p Repositories/SIT
    cd Repositories/SIT
    git clone https://github.com/morse-simulator/morse.git
    cd morse/
    mkdir build && cd build
    cmake .. -DBUILD_ROS_SUPPORT=ON -DPYTHON_EXECUTABLE=/usr/bin/python3.4
    sudo make install
    cd ../..
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
    
puis ajouter les deux lignes suivantes dans le ~/.bashrc:

    export ROS_MASTER_URI=http://ns3002211.ip-37-59-58.eu:11311
    export ROS_IP=$(ip addr | grep 'state UP' -A2 | tail -n1 | awk '{print $2}' | cut -f1 -d'/')

et recharger:

    source ~/.bashrc

Lancer le système
------------------

initialiser roscore sur la vm

    roscore
    
lancer flask sur la vm

    python /dir/toward/project/simulation/simulation.py
    ou
    /dir/toward/project/restart_simulation_ws.sh
    
lancer morse sur le PC:

    /dir/toward/project/startsimulation.sh
    
