The 
auto_environment.sh 
file is supposed to launch 2 terminals, run the 2 "source" lines 
source /opt/ros/jazzy/setup.bash adds ROS and mavros.
source ~/Documents/ros2_ws/install/setup.bash adds your own packages (idt).
and allow the user to decide what launch command to use depending on the hardware setup.
to make it even quicker to run this, you can make a .desktop file as written in the .sh file itself

csv-to-plan 
is the main star of this vibe coded repo(thanks daddy Claude) .
just change the default directories at the top of the program, run it, and the rest should be self explanatory.
(look how to import plans to QGC and especially where that fucker looks for the files and use that as default output dir)
it takes as input a raw csv log and uses the programs Daniele wrote to convert it to a plan ready to upload to QGC.
it doesn't integrate the fixed wing optimization.
To upload plans to QGC just click on the top-left icon >> plan >> open.

REMINDER
rm_outliers I have in this folder is different from the one Daniele wrote and I haven't read it yet.
