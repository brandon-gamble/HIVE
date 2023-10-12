# This works for installing on an ubuntu os I used this image 
# https://ubuntu-mate.org/blog/ubuntu-mate-xenial-final-release/

$ sudo -i
$ sudo apt-get update
$ sudo apt-get upgrade
$ sudo apt install git libssl-dev libusb-1.0-0-dev pkg-config -y
$ sudo apt install cmake python3-dev -y
$ sudo apt-get install build-essential libssl-dev -y
$ sudo apt-get install libglfw3
$ sudo apt-get install libglfw3-dev
$ sudo apt-get install libgl1-mesa-dev libglu1-mesa-dev
$ cd /tmp
$ wget https://github.com/Kitware/CMake/releases/download/v3.20.0/cmake-3.20.0.tar.gz
$ tar -zxvf cmake-3.20.0.tar.gz
$ cd cmake-3.20.0
$ ./bootstrap
$ gmake
$ sudo make install
$ cd ~
$ git clone https://github.com/IntelRealSense/librealsense.git
$ cd librealsense
$ sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/
$ sudo udevadm control --reload-rules && udevadm trigger
$ mkdir build
$ cd build
$ export CC=/usr/bin/gcc
$ export CXX=/usr/bin/g++
$ cmake -D CMAKE_BUILD_TYPE="Release" \            #I dont know why the cmake call needs to be made like this but its apparently important
-D FORCE_LIBUVC=ON \
-D BUILD_PYTHON_BINDINGS=ON \
-D BUILD_EXAMPLES=ON ..
$ make -j4
$ sudo make install
$ sudo ldconfig
$ sudo reboot
