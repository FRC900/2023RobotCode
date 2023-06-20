# Run this to configure a brand new ubuntu install for using our code
# This could be after creating a new VM image from scratch or in a new
# native install
# Should only have to run this once

sudo apt update

# Docker setup
sudo apt install -y curl
curl https://get.docker.com | sh
sudo systemctl --now enable docker

# Allow non-root users to run docker commands
sudo gpasswd -a $USER docker

# Random updates
sudo apt upgrade -y
sudo apt install -y terminator

# log out, log back in (probably also need a restart anyway after apt upgrade)

# Install git-lfs
sudo apt install -y wget
cd &&\
    wget https://github.com/git-lfs/git-lfs/releases/download/v3.2.0/git-lfs-linux-amd64-v3.2.0.tar.gz &&\
	mkdir git-lfs-install &&\
	cd git-lfs-install &&\
	tar -xzf ../git-lfs-linux-amd64-v3.2.0.tar.gz &&\
	cd git-lfs-3.2.0 &&\
	sudo ./install.sh &&\
	cd &&\
	rm -rf git-lfs-linux-amd64-v3.2.0.tar.gz git-lfs-install &&\
	git lfs install

# Set up student's git ssh keys here

# Install repo - perhaps ssh version inside container, worry about SSH keys?
cd 
git clone https://github.com/FRC900/2023RobotCode.git
cd 2023RobotCode
git submodule update --init
docker run --net=host -v /tmp/.X11-unix:/tmp/.X11-unix \
 -v $HOME/2023RobotCode:/home/ubuntu/.2023RobotCode.readonly \
 --ipc=host \
 --shm-size=8G \
 -e DISPLAY=$DISPLAY --privileged --user ubuntu frc900/zebros-2023-dev:latest "wstool update -t /home/ubuntu/2023RobotCode/zebROS_ws/src -j2"

sudo apt autoremove
