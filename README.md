# Turtlebot Move
A ROS based obstacle avoiding turtlebot program. Simulation in Gazebo of turtlebot moving randomly and avoiding obstacles.


## Installations

To install ROS refer [this link](http://wiki.ros.org/ROS/Installation).

## Requirements / Assumptions
#### Project requires an evironment minimum with Ubuntu 18.04, ROS Melodic.
#### System must have turtlebot 3
If not install it using install:
```bash
sudo apt-get install ros-melodic-turtlebot3-*
```


### Step 1. Clone repository and build the package
```bash
git clone --recursive https://github.com/kavyadevd/turtlebotMove.git
cp <repository_path> <catkin_workspace_path/src/>
cd <catkin_workspace>
source ./devel/setup.bash
catkin_make
```

### Step 2. Make sure that a roscore is up and running
```bash
roscore
```

### Step 3. Launch the project using roslaunch
Without rosbag:
```bash
roslaunch turtlebotmove turtlebotmove.launch 
```

With rosbag:
```bash
roslaunch turtlebotmove turtlebotmove.launch rosbag_yn:=true    
```
To compress bag file:
```
rosbag compress *.bag

```
To play rosbag file:
```bash
rosbag play <rosbag-file-path>/<rosbag-file>.bag
```

Bag file is too large for GitHub hence uploaded on google drive link [here](https://drive.google.com/drive/folders/1ZmyV41yVvmUcCoJwuPEF4E94F5UqW0-k?usp=sharing)

## Note
An active operation running on a terminal can be terminated by giving a Ctrl+C input from keyboard at any time.

## Plugins


- CppCheckEclipse

    To install and run cppcheck in Eclipse

    1. In Eclipse, go to Window -> Preferences -> C/C++ -> cppcheclipse.
    Set cppcheck binary path to "/usr/bin/cppcheck".

    2. To run CPPCheck on a project, right-click on the project name in the Project Explorer 
    and choose cppcheck -> Run cppcheck.
    
    3. To run on terminal
    ```bash
    cppcheck --enable=all --std=c++11 -I include/ --suppress=missingIncludeSystem $( find . -name *.cpp -or -name *.h | grep -vE -e "^./build/" -e "^./vendor/") >     Results/cppcheckoutput.txt
    ```
    Results are present at Results/cppcheckoutput.txt
    
- Cpplint
   1. To run cpplint on terminal
   ```bash
   cpplint $( find . -name *.cpp | grep -vE -e "^./build/" -e "^./vendor/") $( find . -name *.hpp | grep -vE -e "^./build/" -e "^./vendor/") >                    Results/cpplintoutput.txt
   ```
   Results are present at Results/cpplintoutput.xml

- Google C++ Style

    To include and use Google C++ Style formatter in Eclipse

    1. In Eclipse, go to Window -> Preferences -> C/C++ -> Code Style -> Formatter. 
    Import [eclipse-cpp-google-style][reference-id-for-eclipse-cpp-google-style] and apply.

    2. To use Google C++ style formatter, right-click on the source code or folder in 
    Project Explorer and choose Source -> Format

[reference-id-for-eclipse-cpp-google-style]: https://raw.githubusercontent.com/google/styleguide/gh-pages/eclipse-cpp-google-style.xml

- Doxygen

    The HTML page for project outlines can be generated using the following commands
    ```bash
    doxygen -g
    doxygen Doxyfile
    ```

