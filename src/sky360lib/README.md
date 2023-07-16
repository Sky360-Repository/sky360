# sky360lib
C++ Library/Apps for background subtraction and other algorithms for sky360 project

# ViBe
Right now the ViBe BS is the only one implemented, you can learn more about ViBe from:

http://www.telecom.ulg.ac.be/publi/publications/barnich/Barnich2011ViBe/index.html

# Getting and Building

* You need a development environment to build with:
  - Build tools (gcc, cmake)
  - OpenCV > 4.0

* Open a terminal:
  ## First we install easy profiler 
  - git clone https://github.com/yse/easy_profiler.git
  - cd easy_profiler
  - mkdir build
  - cd build
  - cmake -DCMAKE_BUILD_TYPE="Release" ..
  - sudo make install

  ## Download and install the qhyccd camera sdk
  - Download qhyccd sdk - https://www.qhyccd.com/html/prepub/log_en.html#!log_en.md
  - tar zxvf sdk_linux64_22.07.06.tgz
  - cd sdk_linux64_22.07.06 
  - sudo bash install.sh
  
  ## Building OpenCV
  - sudo apt install build-essential cmake -y
  - create a new dir:
    - mkdir sky360
    - cd sky360
  - git clone https://github.com/opencv/opencv.git
  - mkdir build
  - cd build
  - cmake ..
  - cmake --build .
  - sudo cmake --install .
  
  ## Building the library/demo
  - go to the sky360 directory
  - git clone https://github.com/Sky360-Repository/sky360lib.git
  - cd embedded-bgsub
  - mkdir build
  - cd build
  - cmake ..
  - cmake --build .
  
  ## Running the demo
  - go to the sky360 directory
  - cd build/bin
  - sky360lib_demo 0
    - The number is the camera number, you might need to change it to 1, 2


# Additional notes for those running Ubuntu in WSL2
The above has been tried with Ubuntu 22.04.1 WSL2. However one in issue came up relating to libcuda.
Problem and workaround is described here: https://github.com/microsoft/WSL/issues/5663#issuecomment-1068499676
cd \Windows\System32\lxss\lib
del libcuda.so
del libcuda.so.1

Note that you might need the following: 
sudo apt install libgtk2.0-dev
If so you have to rerun cmake from OpenCV build directory.
cmake ..
cmake --build .
sudo cmake --install .

# Testing visualization of codebase
[Link to visualization](https://mango-dune-07a8b7110.1.azurestaticapps.net/?repo=Sky360-Repository%2Fsky360lib)
