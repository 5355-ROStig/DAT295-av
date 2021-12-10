# DAT295-av

A project in the course DAT295 - Autonomous and cooperative vehicular systems at Chalmers. We were 7 people in the group
working with two different subprojects using [wifibots](https://www.wifibot.com/):

1. **Cruise control/platooning** using Lidar.
2. **Intersection arbitration** with two bots in a four-way intersection.

## Dependencies
* Tested on Python 3.8.10
* ROS Noetic Ninjemys
  + Only tested on Ubuntu 20.04.3 LTS (Focal Fossa)
* For intersection arbitration:
  + Our patched [GulliView](https://github.com/5355-ROStig/GulliView)


## How to run
I case any other students will work in the EG5355 lab in the future, here's how to run our projects.

Clone this repository onto the wifibots:
```bash
# ask the TA/supervisor for passwords

# Assuming the configuration of the roof-mounted Linksys router hasn't changed
ssh wifitbot@192.168.1.102  # white antenna
git clone git@github.com:5355-ROStig/DAT295-av.git

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
