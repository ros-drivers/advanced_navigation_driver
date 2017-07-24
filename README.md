# Advanced_Navigation_driver
Driver for the range of INS systems from Advanced Navigation



/*********************************************/
Advanced Navigation
ROS Driver
Packet to Published Message Example
Copyright 2017, Advanced Navigation
/*********************************************/


/*********************************************/
Introduction
/*********************************************/
This is an example using the Advanced Navigation Spatial SDK to create a ROS driver that reads and decodes the anpp packets (in this case packet #20 and packet #27) and publishes the information as ROS topics / messages. 

It should work on all Advanced Navigation INS devices.

The code has been written to be easy to understand, so no optimisations have been made.  It should be relatively easy to extend this code to obtain other infomation required from the relevant anpp packet. 

This example has been developed and tested using Ubuntu Linux v16.04 LTS and ROS Lunar. Installation instructions for ROS can be found here: http://wiki.ros.org/lunar/Installation/Ubuntu.

If you require any assistance using this code, please email support@advancednavigation.com.au.


/*********************************************/
Build Instructions
/*********************************************/
ROS driver build instructions from: http://www.clearpathrobotics.com/assets/guides/ros/Creating%20publisher.html#creating-a-workspace-package

1. Open a new Terminal window and type: "mkdir ~/ros[enter]".
2. Type: "mkdir ~/ros/src[enter]".
3. Type: "cd ~/ros/src[enter]".
4. Type: "catkin_init_workspace[enter]".
5. Type: "catkin_create_pkg an_driver roscpp std_msgs[enter]".
6. Copy the supplied "src" directory and the "package.xml" file into the "~/ros/src/an_driver" directory. It should be OK to overwrite the existing "package.xml" file if you are following these instructions explicitly. If installing into your own pre-existing catkin workspace it may be necessary to manually merge the contents of these files.
7. Modify the "package.xml" file in ~/ros/src/an_driver directory as required.
8. Modify the CMakeLists.txt file in the "~/ros/src/an_driver" directory by adding two lines at the end (you may need to modify the paths to suit your installation):
   add_executable(an_driver ~/ros/src/an_driver/src/an_driver.cpp ~/ros/src/an_driver/src/spatial_packets.c ~/ros/src/an_driver/src/an_packet_protocol ~/ros/src/an_driver/src/rs232/rs232.c)
   target_link_libraries(an_driver ${catkin_LIBRARIES})
9. Type: "cd ~/ros[enter]".
10. Type: "catkin_make[enter]".


/*********************************************/
Device Configuration
/*********************************************/
To use this example code, your Advanced Navigation device should be configured to output anpp packets #20 and #27.


/*********************************************/
Run Instructions
/*********************************************/
1. Open a new Terminal window and type: "sudo adduser user_name dialout[enter]". This should only need to be done once.
2. Plug in the USB to RS232 adapter cable that is connected to the Advanced Navigation device.
3. In the Terminal window type: "dmesg | grep tty [enter]" to confirm the name of the com port (should be something like "ttyUSB0").
4. Type: "roscore[enter]" to start ROS Master.
5. Open a new Terminal window and type: "source ~/ros/devel/setup.bash[enter]".
6. Using the name of your specific com port, type: "rosrun an_driver an_driver ttyUYSB0 115200[enter]". 
7. Open a new Terminal window and type: "rosnode list[enter]" to list the available nodes. You should see these listed:
   /an_device_node
   /rosout
8. Type: "rostopic list[enter]" to list the published topics. You should see these listed:
   /an_device/FilterStatus
   /an_device/Imu
   /an_device/NavSatFix
   /an_device/SystemStatus
   /an_device/Twist
9. Type: "rostopic echo /an_device/Imu[enter]"to see the "Imu" messages being published.
10. Open a new Terminal window and type: "rostopic echo /an_device/NavSatFix[enter]"to see the "NavSatFix" messages being published.
11. Open a new Terminal window and type: "rostopic echo /an_device/Twist[enter]"to see the "Twist" messages being published.
12. Open a new Terminal window and type: "rostopic echo /an_device/FilterStatus[enter]"to see the "Filter Status" messages being published.
13. Open a new Terminal window and type: "rostopic echo /an_device/SystemStatus[enter]"to see the "System Status" messages being published.
14. You need to subscribe to these topics in your code to get access to the data in the messages.


/*********************************************/
Published Topics
/*********************************************/
an_device/NavSatFix
an_device/Twist
an_device/Imu
an_device/SystemStatus
an_device/FilterStatus


/*********************************************/
Published Messages: an_device/NavSatFix
/*********************************************/
sensor_msgs / NavSatFix / Header / Stamp / Sec		// Packet 20, Field 3, Unix time
sensor_msgs / NavSatFix / Header / Stamp / Nsec 	// Packet 20, Field 4, Unix time microseconds * 1000
sensor_msgs / NavSatFix / Header / Frame_ID	 		// Fixed to 0
sensor_msgs / NavSatFix / Status / Status			// Packet 20, Field 2, Translated from anpp fix type
sensor_msgs / NavSatFix / Status / Service			// Fixed to 1 (GPS)
sensor_msgs / NavSatFix / Latitude					// Packet 20, Field 5, Converted from radians to degrees 
sensor_msgs / NavSatFix / Longitude					// Packet 20, Field 6, Converted from radians to degrees 
sensor_msgs / NavSatFix / Altitude					// Packet 20, Field 7, Metres
sensor_msgs / NavSatFix / Position_Covariance		// Packet 20, Fields 21, 22, 23, Variance on the diagonals only, converted from standard deviation metres
sensor_msgs / NavSatFix / Position_Covariance_Type	// Fixed to 2 (diagonal known)


/*********************************************/
Published Messages: an_device/Twist
/*********************************************/
geometry_msgs / Twist / Linear / X					// Packet 20, Field 8, Metres per second
geometry_msgs / Twist / Linear / Y					// Packet 20, Field 9, Metres per second
geometry_msgs / Twist / Linear / Z					// Packet 20, Field 10, Metres per second
geometry_msgs / Twist / Angular / X					// Packet 20, Field 18, Radians per second
geometry_msgs / Twist / Angular / Y					// Packet 20, Field 19, Radians per second
geometry_msgs / Twist / Angular / Z					// Packet 20, Field 20, Radians per second


/*********************************************/
Published Messages: an_device/Imu
/*********************************************/
sensor_msgs / Imu / Orientation / X					// Packet 20, Fields 15, 16, 17, Converted from radians to quaternions
sensor_msgs / Imu / Orientation / Y					// Packet 20, Fields 15, 16, 17, Converted from radians to quaternions
sensor_msgs / Imu / Orientation / Z					// Packet 20, Fields 15, 16, 17, Converted from radians to quaternions
sensor_msgs / Imu / Orientation / W					// Packet 20, Fields 15, 16, 17, Converted from radians to quaternions
sensor_msgs / Imu / Orientation_Covariance			// Packet 27, Fields 2, 3, 4, Anpp Packet 27 Field 1 is for the W axis which is not requested
sensor_msgs / Imu / Angular_Velocity / X			// Packet 20, Field 18, Radians per second, same as geometry_msgs / Twist / Angular / X
sensor_msgs / Imu / Angular_Velocity / Y			// Packet 20, Field 19, Radians per second, same as geometry_msgs / Twist / Angular / Y
sensor_msgs / Imu / Angular_Velocity / Z			// Packet 20, Field 20, Radians per second, same as geometry_msgs / Twist / Angular / Z
sensor_msgs / Imu / Linear_Acceleration / X			// Packet 20, Field 11, Metres per second per second
sensor_msgs / Imu / Linear_Acceleration / Y			// Packet 20, Field 12, Metres per second per second
sensor_msgs / Imu / Linear_Acceleration / Z			// Packet 20, Field 13, Metres per second per second


/**********************************************/
Published Messages: an_device/SystemStatus
/**********************************************/
diagnostic_msgs / Diagnostic_Status / Name			// Fixed to "System Status"
diagnostic_msgs / Diagnostic_Status / Level			// Packet 20, Field 1, Set to 0 (OK) or 2 (ERROR)
diagnostic_msgs / Diagnostic_Status / Message		// Packet 20, Field 1, Varies


/**********************************************/
Published Messages: an_device/FilterStatus
/**********************************************/
diagnostic_msgs / Diagnostic_Status / Name			// Fixed to "Filter Status"
diagnostic_msgs / Diagnostic_Status / Level			// Packet 20, Field 2, Set to 0 (OK) or 1 (WARN)
diagnostic_msgs / Diagnostic_Status / Message		// Packet 20, Field 2, Varies


/**********************************************/
Messages Not Published
/**********************************************/
sensor_msgs / Imu / Angular_Velocity_Covariance 
sensor_msgs / Imu / Linear_Acceleration_Covariance
