# Installing MAVSDK (https://mavsdk.mavlink.io/develop/en/contributing/build.html)

- sudo apt-get update -y
- sudo apt-get install cmake build-essential colordiff git doxygen -y

- git clone https://github.com/shakeebbb/MAVSDK.git
- cd MAVSDK
- git checkout master (development is checked out by default)
- git submodule update --init --recursive

- cmake -DCMAKE_BUILD_TYPE=Debug -DBUILD_SHARED_LIBS=ON -Bbuild/default -H.
- cmake --build build/default

- cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON -DCMAKE_INSTALL_PREFIX=install -Bbuild/default -H.
- cmake --build build/default --target install

- export MAVSDK_DIR=<path>/MAVSDK/install/lib/cmake/MAVSDK/
- cd MAVSDK/examples/reboot_autopilot
- mkdir build && cd build
- cmake ..
- make

- ./build/reboot_autopilot serial:///dev/ttyACM0:57600
