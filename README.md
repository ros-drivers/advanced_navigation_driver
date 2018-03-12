# advanced_navigation_driver
Driver for the range of INS systems from Advanced Navigation

Packet to Published Message Example

Copyright 2017, Advanced Navigation

This is an example using the Advanced Navigation Spatial SDK to create a ROS driver that reads and decodes the anpp packets (in this case packet #20 and packet #27) and publishes the information as ROS topics / messages. 

It should work on all Advanced Navigation INS devices.

This example has been developed and tested using Ubuntu Linux v16.04 LTS and ROS Lunar. Installation instructions for ROS can be found here: http://wiki.ros.org/lunar/Installation/Ubuntu.

If you require any assistance using this code, please email support@advancednavigation.com.au.

Installation, build, device configuration, and execution instructions can be found in the file "Advanced Navigation ROS Driver Notes.txt". 


*** modifications by Kyler Laird ***

Orientation now complies with REP 103.  East is zero degrees.  Degrees increment counter-clockwise.
https://github.com/ros-drivers/advanced_navigation_driver/issues/3#issuecomment-372348146

New parameters:

	rtcm:  Specify a topic name.  Strings published to this topic will be passed to the device as RTCM corrections.

	utm_zone:  Specify a UTM Zone number.  This will be used for calculating the transform.

	tf_name:  Specify the name of the transform to broadcast.

These changes are all tentative and need testing.
