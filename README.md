# advanced_navigation_driver
Driver for the range of INS systems from Advanced Navigation

Packet to Published Message Example

Copyright 2017, Advanced Navigation

This is an example using the Advanced Navigation Spatial SDK to create a ROS driver that reads and decodes the anpp packets (in this case packet #20, #27, #36 and #40) and publishes the information as ROS topics / messages.

It should work on all Advanced Navigation INS devices.

This example has been developed and tested using Ubuntu Linux v16.04 LTS and ROS Lunar. Installation instructions for ROS can be found here: http://wiki.ros.org/lunar/Installation/Ubuntu.

If you require any assistance using this code, please email support@advancednavigation.com.au.

Installation, build, device configuration, and execution instructions can be found in the file "Advanced Navigation ROS Driver Notes.txt". 


*** modifications by Kyler Laird ***

Orientation now complies with REP 103.  East is zero degrees.  Degrees increment counter-clockwise.
https://github.com/ros-drivers/advanced_navigation_driver/issues/3#issuecomment-372348146

new parameters:
	utm_zone:  Specify a UTM Zone number.  This will be used for calculating the transform.

new topics:
	rtcm:  Strings published to this topic will be passed to the device as RTCM corrections.
	odom:  Odometry from the device is published to this topic.  (Depends on utm_zone.)

new frames:
	~:  This frame has the name of the current name space and describes the position and orientation of the device.  (Depends on utm_zone.)
	
Under Linux, serial port reads are blocking.  This reduces CPU usage from 100% to 5% (for 20 Hz on an i5).

These changes are all tentative and need testing.
