# source /home/ubuntu/anaconda3/etc/profile.d/conda.sh #go back to conda
#go back to conda
source anaconda3/etc/profile.d/conda.sh 
#fix broken NVIDIA Driver?
https://www.murhabazi.com/install-nvidia-driver
#-------------------------------------------------

# Make sure ROS is sourced. This is the system-level ros you installed
source /opt/ros/noetic/setup.bash

# Set up catkin workspace
cd # go to your home directory
mkdir -p ROS/src # make a directory for your catkin workspace (naming it ROS)
cd ~/ROS # go to your catkin workspace
catkin_make # initialize your catkin workspace. This sets up the directory structure so you can build your own packages.
# source your workspace, adding this workspace to the system ROS packages. (Note: If you put this line in your .bashrc you don't need to source it every time you open a new terminal)
source ~/ROS/devel/setup.bash 

# Now set up our own package for the listener/publisher example
cd ~/catkin_ws/src # go to the source directory
catkin_create_pkg funtimes std_msgs rospy roscpp # create a package called funtimes with dependencies on std_msgs, rospy, and roscpp

# Now we will create the listener and publisher scripts to the package
cd funtimes/src # they're fun!
touch listener.py # create the listener script
touch talker.py # create the publisher script
chmod +x listener.py # make them executable
chmod +x talker.py # make them executable

# Write the code that will go in the listener and publisher scripts

# Now we will build the package so ROS can find it (even though its python so there's no compilation)
cd ~/catkin_ws # go to the catkin workspace
catkin_make # build everything

# Now we can run the listener and publisher scripts. First we have to start the roscore!
roscore
rosrun funtimes listener.py # run the listener
rosrun funtimes talker.py

# check if the funtimes started, if you're not having funtimes, you might not have followed the instructions (joke). 


#tutorial on setting up LIDAR scanner in Gazebo
https://kiranpalla.com/blog/ros-using-gazebo-laser-scan-plug-in/
