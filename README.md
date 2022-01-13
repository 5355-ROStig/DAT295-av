# DAT295-av

A project in the course DAT295 - Autonomous and cooperative vehicular systems at Chalmers. We were 7 people in the group
working with two different subprojects using [wifibots](https://www.wifibot.com/):

1. **Cruise control/platooning** using Lidar.
2. **Intersection arbitration** with two bots in a four-way intersection.

## Dependencies
* Tested on Python 3.8.10
* ROS Noetic Ninjemys
  + Only tested on Ubuntu 20.04.3 LTS (Focal Fossa)
* Our forked [roswifibot driver](https://github.com/5355-ROStig/roswifibot)
* For intersection arbitration:
  + Our patched [GulliView](https://github.com/5355-ROStig/GulliView)


## How to run
I case any other students will work in the EG5355 lab in the future, here's how to run our projects.

Clone this repository and the forked roswifibot driver onto the wifibots:
```bash
# ask the TA/supervisor for passwords

# Assuming the configuration of the roof-mounted Linksys router hasn't changed
ssh wifitbot@192.168.1.102  # white antenna
git clone git@github.com:5355-ROStig/DAT295-av.git

# and the driver
cd DAT295-av
git clone git@github.com:5355-ROStig/roswifibot.git

# repeat for 192.168.1.103 (black antenna)
# and for 192.168.1.101 (lidar bot) if you want to run cruise control stuff
```

For each bot, go to the root of the repository (which is a catkin workspace) and run the catkin make tool.
```bash
cd DAT295-av
catkin_make
```

Remember to source the package files as described
in [the ROS documentation](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment#Managing_Your_Environment)
.
```bash
echo "/opt/ros/noetic/setup.bash" >> ~/.bashrc
# Might need to change to wherever you cloned it
echo "~/DAT295-av/devel/setup.bash" >> ~/.bashrc  
source ~/.bashrc
```

### Braking and Platooning specific

#### Use Case 1

Place two robots one after another with a safe distance. The robots will be in the movement 3 sec and speed 0.2 (can be changed in the code).

For running experiment three files need to be activated.
At front robot:
```bash
rosrun use-case-1.py front
```
At back robot:
```bash
rosrun use-case-1.py back
```

Start robots simultaneously (just on some computer that is contected to Rostig network):
```bash
python init_start.py
```

Finally, measure the distance between the robots after stop and compare with the initial one.



#### Use Case 2

To run this experiment it is enough to use only one robot with IR sensor and one obsticle in front (e.g. cardboard wall) which the robot will be going to. The robot is supposed to stop if the distance to the object is shorter than 50 cm. The speed used in the experiment was 0.2 (can be changed).

Start the robot:
```bash
rosrun use-case-2-py
```
The program will print the distance to the object. 

#### Use Case 3

Place two robots one after another on a safe distance (70 cm was used). Back robot must be robot with a LiDAR-sensor. 

Run firstly the back robot (it will adopt its distance to the front robot):
```bash
rosrun lidar_receiver.py
```

Run front robot:
```bash
rosrun use-case-3-front.py
```


#### Use Case 4

Place robots on the safe distance one after another. 

Run first back robot:
```bash
rosrun use-case-4-back-new.py
```
Then run front robot:
```bash
rosrun use-case-4-front.py
```

### Intersection arbitation specific
First start GulliView on the roof mounted computer:
```bash
ssh -X 192.168.1.122
# Go to the directory of this file
find . -iname "startCamerasBroadcast.sh"
# Run it
./startCamerasBroadcast.sh 2121
```

Then place two bots (only tested on the non lidar ones) on two different roads marked with blue tape. If this tape is gone you can look at
the images in the [mapdata node](/src/mapdata) readme to at least make sense of the code.

Put [AprilTags](https://april.eecs.umich.edu/software/apriltag) on the wifibot plywood mounts, then start the intersection arbitration on each bot.
```bash
# Open two terminals

# Run this on one bot (e.g. one with tag 6)
# For example, a scenario with priority road (huvudled)
roslaunch intercrossing_master wifibot.launch robot:=/ tag_id:=6 scenario='scenario1'

# Run this on the other bot (e.g. one with tag 4)
roslaunch intercrossing_master wifibot.launch robot:=/ tag_id:=4 scenario='scenario1'
```

Look at em go! Now you can log data with our [time-to-collision package](/src/ttc) see the UDP messages being
broadcasted with [our monitor](/src/coordination/src/monitor.py) or use the [visualization package](/src/visualization)
to see the tags moving around in rviz.
