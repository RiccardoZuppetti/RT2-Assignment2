# Research Track II - Second Assignment - Main Branch

In this branch it is possible to find the code to execute the project, without any documentation.

# How to Run

Verify that you have installed ROS Noetic. Create a ROS Workspace and clone this ROS package inside it

```
git clone -b main https://github.com/RiccardoZuppetti/RT2-Assignment2.git rt2_assignment2
```

Then build the workspace using the `catkin_make` command.

Modify the `.bashrc` file, using the command `gedit ~/.bashrc`, sourcing the ROS environment and the ROS workspace.

To run this project open a terminal, move to the created ROS workspace and digit

```
roslaunch rt2_assignment2 sim.launch 
```

Then to open the jupyter notebook, open a second terminal and digit

```
jupyter notebook --allow-root --ip 0.0.0.0
```

From this software open the "user_interface.ipynb" file located in `rt2_assignment2/notebook`
