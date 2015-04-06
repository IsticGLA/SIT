# Projet de SIT, Istic 2015 Génie Logiciel en Apprentissage
Système d'Information Tactique pour les pompier avec gestion de flotte de drones.

# Comment initialiser son environnement ?
**Vagrant et ROS**
Installer vagrant :

    sudo apt-get install -y vagrant
initialiser la machine vagrant :

    vagrant up
se connecter à la machine vagrant

    vagrant ssh
lancer roscore sur la machine vagrant

    roscore
votre noeud ROS est lancé

**Simulateur Morse**
l'installation est obligatoirement manuelle
installation des dépendances :

    sudo apt-get install -y cmake python3 python3-dev blender python3-yaml python3-setuptools build-essential 
Utilisation d'un répertoire temporaire pour les repo nécéssaires

    cd
    mkdir Repositories/SIT
    cd Repositories/SIT
Installations :

    git clone https://github.com/morse-simulator/morse.git
    cd morse/
    mkdir build && cd build
    cmake .. -DBUILD_ROS_SUPPORT=ON -DPYTHON_EXECUTABLE=/usr/bin/python3.4
    sudo make install
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

puis ajouter les deux lignes suivantes dans votre ~/.bashrc :

    export ROS_MASTER_URI=http://ros-indigo-desktop-trusty64:11311
    export ROS_IP=192.168.33.1 # Listen on any interface
faites ensuite dans votre terminal

    source ~/.bashrc
    
# Lancer le système
lancer le noeud ROS avec vagrant (cf ci dessus)
lancer la simulation avec morse

    morse run chemin/vers/fichier.py
    
pour envoyer un topic :

    rostopic pub -1 /dest_point geometry_msgs/Point "{x: 1, y: 1, z: 3}"
