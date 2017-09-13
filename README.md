# advanced_navigation_driver
Driver for the range of INS systems from Advanced Navigation

Packet to Published Message Example

Copyright 2017, Advanced Navigation

This is an example using the Advanced Navigation Spatial SDK to create a ROS driver that reads and decodes the anpp packets (in this case packet #20 and packet #27) and publishes the information as ROS topics / messages. 

It should work on all Advanced Navigation INS devices.

The code has been written to be easy to understand, so no optimisations have been made.  It should be relatively easy to extend this code to obtain other infomation required from the relevant anpp packet. 

This example has been developed and tested using Ubuntu Linux v16.04 LTS and ROS Lunar. Installation instructions for ROS can be found here: http://wiki.ros.org/lunar/Installation/Ubuntu.

If you require any assistance using this code, please email support@advancednavigation.com.au.

Installation, build, device configuration, and execution instructions can be found in the file "Advanced Navigation ROS Driver Notes.txt". 

