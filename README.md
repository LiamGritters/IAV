# IAV
Integrated Autonomous Vehicle (IAV)

This is the official github for the Carleton Integrated Autonomous Vehicle (IAV) Capstone Project

## Dependencies 
- opencv (version 2.4 currently supported)
- lcm (https://lcm-proj.github.io/index.html)
- build-essential
- libglib2.0-dev
- libncurses5-dev
- libudev-dev

Install these dependencies before attempting to build 
```
sudo apt-get install libopencv-dev build-essential libglib2.0-dev libncurses5-dev
git clone https://github.com/lcm-proj/lcm lcm
cd lcm
mkdir build
cd build
cmake ..
make
sudo make install
```

## Building 
Currently the only supported platform is Linux. You must also have a complier that supports C++11

clone this repository  
```
git clone https://github.com/LiamGritters/IAV.git
```

An environment variable must be made to point to your workspace 
```
sudo gedit /etc/environment
```
Add the following to the file:
```
IAV_WORKSPACE="{PATH_TO_IAV_DIRECTORY}"
```
where {PATH_TO_IAV_DIRECTORY} is the location of the IAV directory cloned from here. 
For instance: 
```
IAV_WORKSPACE="/home/usr/workspace/IAV"
```
You need to logout from the current user and login again so that the changes take place.

navigate back to the IAV directory 
```
make
```

## Simulator
Download Webots simulator: https://cyberbotics.com/#download

Open the World: File->Open World-> IAV/utilities/webots/worlds/carleton.wbt

Build the project (F7)

There are utilties to interact with the vehicle and display data that can be found at: /IAV/components/ 



