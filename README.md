# Allegro_Toolkit

## Dependencies:

- Allegro Hand ROS:
1) clone the package from the repo:
```bash
$ git clone https://github.com/yias/allegro-hand-ros.git
```

- RBDL: 

1) clone the package from the repo: 
```bash
$ git clone https://github.com/ORB-HD/rbdl-orb.git
$ cd rbdl-orb
```
2) create a build folder, compile the package with the RBDL_BUILD_ADDON_URDFREADER flag set to ON and install the package
```bash
$ mkdir build
$ cd build/
$ cmake -D CMAKE_BUILD_TYPE=Release -D RBDL_BUILD_ADDON_URDFREADER=ON ..
$ make
$ sudo make install
$ sudo ldconfig
```


- qbOASES: 

1) clone the package from the repo: 
```bash
$ git clone https://github.com/coin-or/qpOASES.git
$ cd qpOASES
```
2) create a build folder, compile and install the package
```bash
$ mkdir build
$ cd build
$ cmake ..
$ sudo make install
$ sudo ldconfig
```


```

## Running the package:
There are multiple controllers available:

1) Passive ds:
```bash
$ roslaunch allegro_toolkit ds_force.launch
```

2) fingertip control (receiving finger postions from external package):
```bash
$ roslaunch allegro_toolkit ft_controller.launch
```


3) QP controller (there is a bug, has to be fixed):
```bash
$ roslaunch allegro_toolkit qp_controller.launch
```

4) adaptive controller :
```bash
$ roslaunch allegro_toolkit adaptive_controller.launch
```