These are the steps I followed to (try to) get the realsense running on a Raspi 4.

# Imaging
- Go to [Ubuntu Mate webpage](https://ubuntu-mate.org/blog/ubuntu-mate-xenial-final-release/)
- Download raspi 64 bit Ubuntu MATE 22.04.1 LTS
- Use Raspberry Pi Imager to burn image onto sd

# Pre-installation
- Follow instructions on [github wiki](https://github.com/datasith/Ai_Demos_RPi/wiki/Raspberry-Pi-4-and-Intel-RealSense-D435)
```
sudo apt-get update && sudo apt-get dist-upgrade
sudo apt-get install automake libtool vim cmake libusb-1.0-0-dev libx11-dev xorg-dev libglu1-mesa-dev
```

- Expand the filesystem by selecting the `Advanced Options` menu entry, and select yes to rebooting:
```
sudo raspi-config
```

- Install `swapfile`
```
sudo apt-get install -y dphys-swapfile
```

- Increase swap to 2GB by changing the file below to `CONF_SWAPSIZE=2048`:
```
sudo vi /etc/dphys-swapfile
```
To save and quit vi: press <kbd>ESC</kbd> key to switch from `INSERT` mode to `COMMAND` mode, then type `:wq` to write file to disk and quit vi.

- Apply the change:
```
sudo /etc/init.d/dphys-swapfile restart swapon -s
```

- Install `git`
```
sudo apt install git
```

- Create new `udev` rule:
```
cd ~
git clone https://github.com/IntelRealSense/librealsense.git
cd librealsense
sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/ 
```

- Apply change (needs to be run by root):
```
sudo su
udevadm control --reload-rules && udevadm trigger
exit
```

- Modify the path by adding the following line to the `.bashrc` file:
```
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
```

- Apply the change:
```
source ~/.bashrc
```

# Installation

- Install `protobuf` - Google's language-neutral, platform-neutral, extensible mechanism for serializing structured data:
```
cd ~
git clone --depth=1 -b v3.10.0 https://github.com/google/protobuf.git
cd protobuf
./autogen.sh
./configure
make -j1
sudo make install
cd python
export LD_LIBRARY_PATH=../src/.libs
sudo apt install python3-pip
python3 setup.py build --cpp_implementation 
python3 setup.py test --cpp_implementation
# error here. :(





# References
- https://dev.intelrealsense.com/docs/using-depth-camera-with-raspberry-pi-3
- https://ubuntu-mate.org/blog/ubuntu-mate-xenial-final-release/
- https://github.com/datasith/Ai_Demos_RPi/wiki/Raspberry-Pi-4-and-Intel-RealSense-D435
