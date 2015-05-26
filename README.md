# Projet de SIT, Istic 2015 Génie Logiciel en Apprentissage
Système d'Information Tactique pour les pompier avec gestion de flotte de drones.

# Comment initialiser son environnement ?
ROS
-------------------

Le noeud master ROS doit être installé sur une machine permettant une connection bidirectionnelle noeud <-> PC hébergeant la simulation morse, et noeud <-> serveur hébergeant nivimoju
Dans notre cas, cette machine est une VM hébergée à l'ISTIC.
    
Installation de ROS sur la VM :
--------------------------------

    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list'
    wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -
    sudo apt-get update
    sudo apt-get install xserver-xorg-dev-lts-utopic mesa-common-dev-lts-utopic libxatracker-dev-lts-utopic libopenvg1-mesa-dev-lts-utopic libgles2-mesa-dev-lts-utopic libgles1-mesa-dev-lts-utopic libgl1-mesa-dev-lts-utopic libgbm-dev-lts- utopic libegl1-mesa-dev-lts-utopic
    sudo apt-get install ros-indigo-ros-base
    sudo rosdep init
    rosdep update
    echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
    echo "export ROS_IP=XX.XX.XX.XX # ip de la machine" >> ~/.bashrc
    echo "export ROS_MASTER_URI=http://ip.de.la.vm:11311" >> ~/.bashrc
    source ~/.bashrc
    sudo apt-get install python-rosinstall

Installation d'opencv et flask sur la VM (python qui commanderas le drone et enverra des images)
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

Installation de MORSE sur le PC faisant tourner la simulation
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

    export ROS_MASTER_URI=http://ip.de.la.vm.de.l.ISTIC:11311
    export ROS_IP=$(ip addr | grep 'state UP' -A2 | tail -n1 | awk '{print $2}' | cut -f1 -d'/')

et recharger:

    source ~/.bashrc
    
Noeuds ROS et configuration
=======================

Il y a 3 noeuds pour ROS :

roscore
-------
roscore, qui tourne sur le serveur.
Pour le lancer :

    nohup /etc/init.d/roscore start &

ce noeud doit être lancé en premier

flask
-----
le second noeud est /node_server et il correspond au webservice flask qui lira les position et enverra les ordres à la simulation. Il est présent sur le serveur
Pour le lancer :

    ./restart_simulation_ws.sh

les logs de flask sont visibles dans /sit/log/simulation_ws.log
Pour changer la configuration du serveur pour le client qui va appeller le webservice de flask, changer le fichier simulation-client/resources/config.properties

morse
-----
le troisième noeud est celui de morse (la simulation). Il est lancé en local sur le PC.
Pour le lancer

    ./startsimulation.sh

Troubleshooting
===============
référence pour comprendre les noeuds ROS : https://vimeo.com/67806888
http://roscon.ros.org/2013/wp-content/uploads/2013/06/Networking-for-ROS-Users.pdf

Commandes utiles
----------------

### rqt_graph

Si quelque chose ne marche pas, la première commande utile est

    rqt_graph

elle montre un graphe avec les noeuds et subsriber associés. Si un noeud est en rouge c'est qu'il y a une erreur de configuration dans les ROS_IP. Les commandes suivante permettent de determiner pourquoi. Le graphe habituel devrait ressembler à ça :
 
![graphe ros](https://media.taiga.io/attachments/e/6/8/f/d86a4112ad0f9969f9d3c68d0f7142025e79d6fd740e3d71863fa545a28b/captured.png "Graphe habituel")

### rosnode info

    rosnode info morse
    rosnode info node_server

Permet de détecter les erreurs de configuration dans les IP

Couchbase
-------------------

L'installation de Couchbase se fait dans notre cas sur un serveur distant. Il peut se faire sur la même machine ou tourne le noeud ROS master mais pour des raisons de performances (Couchbase nécessitant beaucoup de RAM > 8Go), nous avons décidé de faire l'installation de Couchbase sur une machine dédiée à cette base de donnée.

Pour installer Couchbase, il faut passer par le .deb fournit sur le site de Couchbase :

http://www.couchbase.com/nosql-databases/downloads

Cliquer sur la version de votre choix, remplissez le formulaire et sauvegarder le .deb sur votre machine.

Pour l'installer il faut taper la commande pour installer le .deb :
    
    sudo dpkg --instdir=/opt/couchbase -i couchbase-server-version.deb

Une fois l'installation terminée, Couchbase démarre automatiquement. Il faut maintenant se rendre sur l'ip de votre machine distance où est installé Couchbase, pour faire la première configuration de la base de donnée.
    
    http://ip.de.votre.machine:8091

Rentrez les informations demandées et lors de la création du bucket, mettez le nom sit_bucket et laissez les paramètres par défauts.
Sur la page suivante, un login et un mot de passe vous sont demandés. Ces derniers vous serviront à vous connecté sur l'interface web de Couchbase.

Il ne reste plus qu'a populer la base de donnée avec les données statiques, indispensable à l'application.
Pour cela, taper cette suite de commande à partir du repertoire ou vous avez cloner le git :

    /opt/couchbase/bin/cbrestore -u Administrator -p password /SIT/database/ couchbase://localhost:8091 --bucket-source=sit_bucket