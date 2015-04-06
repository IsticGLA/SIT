# -*- mode: ruby -*-
# vi: set ft=ruby :

# All Vagrant configuration is done below. The "2" in Vagrant.configure
# configures the configuration version (we support older styles for
# backwards compatibility). Please don't change it unless you know what
# you're doing.
Vagrant.configure(2) do |config|
  # The most common configuration options are documented and commented below.
  # For a complete reference, please see the online documentation at
  # https://docs.vagrantup.com.

  # Every Vagrant development environment requires a box. You can search for
  # boxes at https://atlas.hashicorp.com/search.
  config.vm.box = "shadowrobot/ros-indigo-desktop-trusty64"

  # Disable automatic box update checking. If you disable this, then
  # boxes will only be checked for updates when the user runs
  # `vagrant box outdated`. This is not recommended.
  # config.vm.box_check_update = false

  # Create a forwarded port mapping which allows access to a specific port
  # within the machine from a port on the host machine. In the example below,
  # accessing "localhost:8080" will access port 80 on the guest machine.
  config.vm.network "forwarded_port", guest: 11311, host: 11311
  config.vm.network "forwarded_port", guest: 5000, host: 5000
  #config.vm.network "forwarded_port", guest: 80, host: 80
  #config.vm.network "forwarded_port", guest: 80, host: 80, protocol: "udp"

  # Create a private network, which allows host-only access to the machine
  # using a specific IP.
  config.vm.network "private_network", ip: "192.168.33.10"

  # Create a public network, which generally matched to bridged network.
  # Bridged networks make the machine appear as another physical device on
  # your network.
  #config.vm.network "public_network"

  # Share an additional folder to the guest VM. The first argument is
  # the path on the host to the actual folder. The second argument is
  # the path on the guest to mount the folder. And the optional third
  # argument is a set of non-required options.
  config.vm.synced_folder "../simulation", "/simulation"

  # Provider-specific configuration so you can fine-tune various
  # backing providers for Vagrant. These expose provider-specific options.
  # Example for VirtualBox:
  #
  # config.vm.provider "virtualbox" do |vb|
  #   # Display the VirtualBox GUI when booting the machine
  #   vb.gui = true
  #
  #   # Customize the amount of memory on the VM:
  #   vb.memory = "1024"
  # end
  #
  # View the documentation for the provider you are using for more
  # information on available options.

  # Define a Vagrant Push strategy for pushing to Atlas. Other push strategies
  # such as FTP and Heroku are also available. See the documentation at
  # https://docs.vagrantup.com/v2/push/atlas.html for more information.
  # config.push.define "atlas" do |push|
  #   push.app = "YOUR_ATLAS_USERNAME/YOUR_APPLICATION_NAME"
  # end

  # Enable provisioning with a shell script. Additional provisioners such as
  # Puppet, Chef, Ansible, Salt, and Docker are also available. Please see the
  # documentation for more information about their specific syntax and use.
  config.vm.provision "shell", inline: <<-SHELL
	sudo apt-get update

	sudo apt-get install -y python3
	sudo apt-get install -y python3-pip
	sudo apt-get install python3-dev

	sudo pip3 install flask-restful
	
	cd ~
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
	git clone git://github.com/ros-infrastructure/catkin_pkg.git -b 0.1.9
	cd catkin_pkg
	sudo python3 setup.py install
	cd ..
	git clone git://github.com/ros/catkin.git
	cd catkin
	sudo python3 setup.py install
	echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
	echo "export ROS_IP=192.168.33.10" >> ~/.bashrc
	source ~/.bashrc  SHELL
end
