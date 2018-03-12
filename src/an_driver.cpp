/****************************************************************/
/*                                                              */
/*          Advanced Navigation Packet Protocol Library         */
/*        ROS Driver, Packet to Published Message Example       */
/*          Copyright 2017, Advanced Navigation Pty Ltd         */
/*                                                              */
/****************************************************************/
/*
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

/*
2018.02.09  Kyler Laird
added RTCM handling, configuration parameters

2018.03.10  Kyler Laird
added REP compliance, transform
*/

#include <ros/ros.h>
#include <ros/serialization.h>
#include <tf/transform_broadcaster.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <unistd.h>
#include <string.h>

#include "rs232/rs232.h"
#include "an_packet_protocol.h"
#include "spatial_packets.h"

#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/TimeReference.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>

#define RADIANS_TO_DEGREES (180.0/M_PI)


int an_packet_transmit(an_packet_t *an_packet)
{
        an_packet_encode(an_packet);
	return SendBuf(an_packet_pointer(an_packet), an_packet_size(an_packet));
}

void set_filter_options()
{
        an_packet_t *an_packet;
        filter_options_packet_t filter_options_packet;

        /* initialise the structure by setting all the fields to zero */
        memset(&filter_options_packet, 0, sizeof(filter_options_packet_t));

        filter_options_packet.permanent = TRUE;
        filter_options_packet.vehicle_type = vehicle_type_car;
        filter_options_packet.internal_gnss_enabled = TRUE;
        filter_options_packet.atmospheric_altitude_enabled = TRUE;
        filter_options_packet.velocity_heading_enabled = TRUE;
        filter_options_packet.reversing_detection_enabled = TRUE;
        filter_options_packet.motion_analysis_enabled = TRUE;

        an_packet = encode_filter_options_packet(&filter_options_packet);

        an_packet_transmit(an_packet);

        an_packet_free(&an_packet);
}

void set_filter_options_x()
{
	//an_packet_t *an_packet = an_packet_allocate(17, 186);

	an_packet_t *an_packet = an_packet_allocate(4, 55);
	memcpy(&an_packet->data[0], "test", 5 * sizeof(uint8_t));
        an_packet_transmit(an_packet);
        an_packet_free(&an_packet);
}

void handle_rtcm(const std_msgs::String::ConstPtr& msg) {
	const char *rtcm_data;
	uint32_t string_length = msg->data.length();

	// ROS_INFO("RTCM: %d bytes",  string_length);
	
	an_packet_t *an_packet = an_packet_allocate(string_length, packet_id_rtcm_corrections);
	memcpy(&an_packet->data[0], &msg->data[0], string_length);
	an_packet_transmit(an_packet);
	an_packet_free(&an_packet);
}


// LatLong-UTM.c++
// Conversions:  LatLong to UTM;  and UTM to LatLong;
// by Eugene Reimer, ereimer@shaw.ca, 2002-December;
// with LLtoUTM & UTMtoLL routines based on those by Chuck Gantz chuck.gantz@globalstar.com;
// with ellipsoid & datum constants from Peter H Dana website (http://www.colorado.edu/geography/gcraft/notes/datum/edlist.html);
//
// Usage:  see the Usage() routine below;
//
// Copyright © 1995,2002,2010 Eugene Reimer, Peter Dana, Chuck Gantz.  Released under the GPL;  see http://www.gnu.org/licenses/gpl.html
// (Peter Dana's non-commercial clause precludes using the LGPL)


#include <cmath>			//2010-08-11: was <math.h>	  
#include <cstdio>			//2010-08-11: was <stdio.h>	  

#define fr 298.257223563
#define a 6378137
#define k0 0.9996
const double PI       =	4*atan(1);	//Gantz used: PI=3.14159265;
const double deg2rad  = PI/180;
const double ee = 2/fr-1/(fr*fr);
const double EE = ee/(1-ee);
double LongOriginRad;

void LLtoUTM(int Zone, double LatRad, double LongRad,  double& Northing, double& Easting) {
	// converts LatLong to UTM coords;  3/22/95: by ChuckGantz chuck.gantz@globalstar.com, from USGS Bulletin 1532.
	double N, T, C, A, M;

	N = a/sqrt(1-ee*sin(LatRad)*sin(LatRad));
	T = tan(LatRad)*tan(LatRad);
	C = EE*cos(LatRad)*cos(LatRad);
	A = cos(LatRad)*(LongRad-LongOriginRad);

	M= a*((1 - ee/4    - 3*ee*ee/64 - 5*ee*ee*ee/256  ) *LatRad 
	    - (3*ee/8 + 3*ee*ee/32 + 45*ee*ee*ee/1024) *sin(2*LatRad)
	    + (15*ee*ee/256 + 45*ee*ee*ee/1024	  ) *sin(4*LatRad)
	    - (35*ee*ee*ee/3072			  ) *sin(6*LatRad));

	Easting = k0*N*(A+(1-T+C)*A*A*A/6+(5-18*T+T*T+72*C-58*EE)*A*A*A*A*A/120) + 500000.0;

	Northing = k0*(M+N*tan(LatRad)*(A*A/2+(5-T+9*C+4*C*C)*A*A*A*A/24
			    + (61-58*T+T*T+600*C-330*EE)*A*A*A*A*A*A/720));
}






int main(int argc, char *argv[]) {
	// Set up ROS node //
	ros::init(argc, argv, "an_device");
	ros::NodeHandle nh("~");

	// Set up the COM port
	std::string com_port_s;
	nh.param<std::string>("port", com_port_s, "/dev/ttyS0");
	char *com_port = (char *)com_port_s.c_str();

	int baud_rate;
	nh.param<int>("baud", baud_rate, 115200);

	if (OpenComport(com_port, baud_rate))
	{
		ROS_INFO("Could not open serial port %s at %d baud.", com_port, baud_rate);
		exit(EXIT_FAILURE);
	}
	ROS_INFO("port:%s@%d", com_port, baud_rate);

	// If an RTCM topic is provided, subscribe to it and pass corrections to device.
	std::string rtcm_topic;
	ros::Subscriber rtcm_sub;
	if (nh.getParam("rtcm_topic", rtcm_topic)) {
		rtcm_sub = nh.subscribe(rtcm_topic.c_str(), 1000, handle_rtcm);
		ROS_INFO("listening for RTCM on %s", rtcm_topic.c_str());
	}

	// If a UTM Zone is provided, publish transforms.
	// The zone is static to avoid problems due to changing when near a Zone boundary.
	int utm_zone;
	std::string tf_name;
	tf::Transform transform;
	if (nh.getParam("utm_zone", utm_zone)) {
		LongOriginRad = (utm_zone*6 - 183) * deg2rad;
		nh.param<std::string>("tf_name", tf_name, "an_device");  // The default should be chosen better.
		ROS_INFO("using UTM Zone %d to publish transform %s", utm_zone, tf_name.c_str());

	}


	// Initialise Publishers and Topics //
	ros::Publisher nav_sat_fix_pub=nh.advertise<sensor_msgs::NavSatFix>("NavSatFix",10);
	ros::Publisher twist_pub=nh.advertise<geometry_msgs::Twist>("Twist",10);
	ros::Publisher imu_pub=nh.advertise<sensor_msgs::Imu>("Imu",10);
	ros::Publisher system_status_pub=nh.advertise<diagnostic_msgs::DiagnosticStatus>("SystemStatus",10);
	ros::Publisher filter_status_pub=nh.advertise<diagnostic_msgs::DiagnosticStatus>("FilterStatus",10);
	ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);

	// Initialise messages
	sensor_msgs::NavSatFix nav_sat_fix_msg;
	nav_sat_fix_msg.header.stamp.sec=0;
	nav_sat_fix_msg.header.stamp.nsec=0;
	nav_sat_fix_msg.header.frame_id='0'; // fixed
	nav_sat_fix_msg.status.status=0;
	nav_sat_fix_msg.status.service=1; // fixed to GPS
	nav_sat_fix_msg.latitude=0.0;
	nav_sat_fix_msg.longitude=0.0;
	nav_sat_fix_msg.altitude=0.0;
	nav_sat_fix_msg.position_covariance={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
	nav_sat_fix_msg.position_covariance_type=2; // fixed to variance on the diagonal

	geometry_msgs::Twist twist_msg;
	twist_msg.linear.x=0.0;
	twist_msg.linear.y=0.0;
	twist_msg.linear.z=0.0;
	twist_msg.angular.x=0.0;
	twist_msg.angular.y=0.0;
	twist_msg.angular.z=0.0;

	sensor_msgs::Imu imu_msg;
	imu_msg.header.stamp.sec=0;
	imu_msg.header.stamp.nsec=0;
	imu_msg.header.frame_id='0'; // fixed
	imu_msg.orientation.x=0.0;
	imu_msg.orientation.y=0.0;
	imu_msg.orientation.z=0.0;
	imu_msg.orientation.w=0.0;
	imu_msg.orientation_covariance={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
	imu_msg.angular_velocity.x=0.0;
	imu_msg.angular_velocity.y=0.0;
	imu_msg.angular_velocity.z=0.0;
	imu_msg.angular_velocity_covariance={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0}; // fixed
	imu_msg.linear_acceleration.x=0.0;
	imu_msg.linear_acceleration.y=0.0;
	imu_msg.linear_acceleration.z=0.0;
	imu_msg.linear_acceleration_covariance={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0}; // fixed
	
	diagnostic_msgs::DiagnosticStatus system_status_msg;
	system_status_msg.level = 0; // default OK state
	system_status_msg.name = "System Status";
	system_status_msg.message = "";
	
	diagnostic_msgs::DiagnosticStatus filter_status_msg;
	filter_status_msg.level = 0; // default OK state
	filter_status_msg.name = "Filter Status";
	filter_status_msg.message = "";
	
	// get data from com port //
	an_decoder_t an_decoder;
	an_packet_t *an_packet;
	system_state_packet_t system_state_packet;
	quaternion_orientation_standard_deviation_packet_t quaternion_orientation_standard_deviation_packet;
	int bytes_received;
	

	an_decoder_initialise(&an_decoder);


	// Loop continuously, polling for packets
	while (ros::ok())
	{
		if ((bytes_received = PollComport(an_decoder_pointer(&an_decoder), an_decoder_size(&an_decoder))) > 0)
		{
			// increment the decode buffer length by the number of bytes received //
			an_decoder_increment(&an_decoder, bytes_received);
			
			// decode all the packets in the buffer //
			while ((an_packet = an_packet_decode(&an_decoder)) != NULL)
			{
				// acknowledgement packet //
				if (an_packet->id == 0) 
				{
					ROS_INFO("acknowledgement data: %d", an_packet->data[3]);
				}

				// receiver information packet //
				if (an_packet->id == 69) 
				{
					ROS_INFO("receiver information: %d", an_packet->data[0]);
				}

				// system state packet //
				if (an_packet->id == packet_id_system_state) 
				{
					if(decode_system_state_packet(&system_state_packet, an_packet) == 0)
					{	
						// NavSatFix
						nav_sat_fix_msg.header.stamp.sec=system_state_packet.unix_time_seconds;
						nav_sat_fix_msg.header.stamp.nsec=system_state_packet.microseconds*1000;
						if ((system_state_packet.filter_status.b.gnss_fix_type == 1) ||  // 2D
							(system_state_packet.filter_status.b.gnss_fix_type == 2))   // 3D
						{
							nav_sat_fix_msg.status.status=0; // no fix
						}
						else if ((system_state_packet.filter_status.b.gnss_fix_type == 3) ||  // SBAS
							 (system_state_packet.filter_status.b.gnss_fix_type == 5))   // Omnistar/Starfire
						{
							nav_sat_fix_msg.status.status=1; // SBAS
						}
						else if ((system_state_packet.filter_status.b.gnss_fix_type == 4) ||  // differential
							 (system_state_packet.filter_status.b.gnss_fix_type == 6) ||  // RTK float
							 (system_state_packet.filter_status.b.gnss_fix_type == 7))    // RTK fixed
						{
							nav_sat_fix_msg.status.status=2; // GBAS
						}
						else 
						{
							nav_sat_fix_msg.status.status=-1;
						}
						nav_sat_fix_msg.latitude=system_state_packet.latitude * RADIANS_TO_DEGREES;
						nav_sat_fix_msg.longitude=system_state_packet.longitude * RADIANS_TO_DEGREES;
						nav_sat_fix_msg.altitude=system_state_packet.height;
						nav_sat_fix_msg.position_covariance={pow(system_state_packet.standard_deviation[1],2), 0.0, 0.0,
							0.0, pow(system_state_packet.standard_deviation[0],2), 0.0,
							0.0, 0.0, pow(system_state_packet.standard_deviation[2],2)};
	
						// Twist
						twist_msg.linear.x=system_state_packet.velocity[0];
						twist_msg.linear.y=system_state_packet.velocity[1];
						twist_msg.linear.z=system_state_packet.velocity[2];
						twist_msg.angular.x=system_state_packet.angular_velocity[0];
						twist_msg.angular.y=system_state_packet.angular_velocity[1];
						twist_msg.angular.z=system_state_packet.angular_velocity[2];
						
						// IMU
						imu_msg.header.stamp.sec=system_state_packet.unix_time_seconds;
						imu_msg.header.stamp.nsec=system_state_packet.microseconds*1000;

						// Convert roll, pitch, yaw from radians to quaternion format //
						tf::Quaternion orientation;
						orientation.setRPY(
							system_state_packet.orientation[0],
							system_state_packet.orientation[1],
							PI / 2.0f - system_state_packet.orientation[2] // REP 103
						);
						imu_msg.orientation.x = orientation[0];
						imu_msg.orientation.y = orientation[1];
						imu_msg.orientation.z = orientation[2];
						imu_msg.orientation.w = orientation[3];

						imu_msg.angular_velocity.x=system_state_packet.angular_velocity[0]; // These the same as the TWIST msg values
						imu_msg.angular_velocity.y=system_state_packet.angular_velocity[1];
						imu_msg.angular_velocity.z=system_state_packet.angular_velocity[2];
						imu_msg.linear_acceleration.x=system_state_packet.body_acceleration[0];
						imu_msg.linear_acceleration.y=system_state_packet.body_acceleration[1];
						imu_msg.linear_acceleration.z=system_state_packet.body_acceleration[2];

						// transform
						if (utm_zone) {
							static tf::TransformBroadcaster transform_br;
							double N, E;

							LLtoUTM(
								utm_zone,
								system_state_packet.latitude,
								system_state_packet.longitude,
								N, E
							);

							transform.setOrigin(tf::Vector3(
								E, N, system_state_packet.height
								// 0, 0, 0  // move to origin for testing
								// E - 483713.0, N - 4526330.0, 0  // move to origin for testing
							));

							transform.setRotation(orientation);

							transform_br.sendTransform(tf::StampedTransform(
								transform,
								//ros::Time::now(),
								ros::Time(
									system_state_packet.unix_time_seconds,
									system_state_packet.microseconds*1000
								),
								"world",  // Is it reasonable to hardcode this?
								tf_name
							));
							//printf("time: %d\n", system_state_packet.microseconds);

							nav_msgs::Odometry odom;
							odom.header.stamp.sec=system_state_packet.unix_time_seconds;
							odom.header.stamp.nsec=system_state_packet.microseconds*1000;

							odom.header.frame_id= "world"; // fix this!
							odom.child_frame_id = "ins";  // fix this!

							//set the position
							odom.pose.pose.position.x = E;
							odom.pose.pose.position.y = N;
							odom.pose.pose.position.z = system_state_packet.height;
							odom.pose.pose.orientation.x = orientation[0];
							odom.pose.pose.orientation.y = orientation[1];
							odom.pose.pose.orientation.z = orientation[2];
							odom.pose.pose.orientation.w = orientation[3];

							// Is this correct???
							odom.pose.covariance={
								pow(system_state_packet.standard_deviation[1],2), 0.0, 0.0,
								0.0, pow(system_state_packet.standard_deviation[0],2), 0.0,
								0.0, 0.0, pow(system_state_packet.standard_deviation[2],2)
							};

							//set the velocity
							odom.twist.twist.linear.x = system_state_packet.velocity[0];
							odom.twist.twist.linear.y = system_state_packet.velocity[1];
							odom.twist.twist.linear.z = system_state_packet.velocity[2];

							// Set angular velocity.
							odom.twist.twist.angular.x = system_state_packet.angular_velocity[0];
							odom.twist.twist.angular.y = system_state_packet.angular_velocity[1];
							odom.twist.twist.angular.z = system_state_packet.angular_velocity[2];

							//publish the message
							odom_pub.publish(odom);

						}

						// System Status	
						system_status_msg.message = "";
						system_status_msg.level = 0; // default OK state
						if (system_state_packet.system_status.b.system_failure) {
							system_status_msg.level = 2; // ERROR state
							system_status_msg.message = system_status_msg.message + "0. System Failure! ";
						}
						if (system_state_packet.system_status.b.accelerometer_sensor_failure) {
							system_status_msg.level = 2; // ERROR state
							system_status_msg.message = system_status_msg.message + "1. Accelerometer Sensor Failure! ";
						}
						if (system_state_packet.system_status.b.gyroscope_sensor_failure) {
							system_status_msg.level = 2; // ERROR state
							system_status_msg.message = system_status_msg.message + "2. Gyroscope Sensor Failure! ";
						}
						if (system_state_packet.system_status.b.magnetometer_sensor_failure) {
							system_status_msg.level = 2; // ERROR state
							system_status_msg.message = system_status_msg.message + "3. Magnetometer Sensor Failure! ";
						}
						if (system_state_packet.system_status.b.pressure_sensor_failure) {
							system_status_msg.level = 2; // ERROR state
							system_status_msg.message = system_status_msg.message + "4. Pressure Sensor Failure! ";
						}
						if (system_state_packet.system_status.b.gnss_failure) {
							system_status_msg.level = 2; // ERROR state
							system_status_msg.message = system_status_msg.message + "5. GNSS Failure! ";
						}
						if (system_state_packet.system_status.b.accelerometer_over_range) {
							system_status_msg.level = 2; // ERROR state
							system_status_msg.message = system_status_msg.message + "6. Accelerometer Over Range! ";
						}
						if (system_state_packet.system_status.b.gyroscope_over_range) {
							system_status_msg.level = 2; // ERROR state
							system_status_msg.message = system_status_msg.message + "7. Gyroscope Over Range! ";
						}
						if (system_state_packet.system_status.b.magnetometer_over_range) {
							system_status_msg.level = 2; // ERROR state
							system_status_msg.message = system_status_msg.message + "8. Magnetometer Over Range! ";
						}
						if (system_state_packet.system_status.b.pressure_over_range) {
							system_status_msg.level = 2; // ERROR state
							system_status_msg.message = system_status_msg.message + "9. Pressure Over Range! ";
						}
						if (system_state_packet.system_status.b.minimum_temperature_alarm) {
							system_status_msg.level = 2; // ERROR state
							system_status_msg.message = system_status_msg.message + "10. Minimum Temperature Alarm! ";
						}
						if (system_state_packet.system_status.b.maximum_temperature_alarm) {
							system_status_msg.level = 2; // ERROR state
							system_status_msg.message = system_status_msg.message + "11. Maximum Temperature Alarm! ";
						}
						if (system_state_packet.system_status.b.low_voltage_alarm) {
							system_status_msg.level = 2; // ERROR state
							system_status_msg.message = system_status_msg.message + "12. Low Voltage Alarm! ";
						}
						if (system_state_packet.system_status.b.high_voltage_alarm) {
							system_status_msg.level = 2; // ERROR state
							system_status_msg.message = system_status_msg.message + "13. High Voltage Alarm! ";
						}
						if (system_state_packet.system_status.b.gnss_antenna_disconnected) {
							system_status_msg.level = 2; // ERROR state
							system_status_msg.message = system_status_msg.message + "14. GNSS Antenna Disconnected! ";
						}
						if (system_state_packet.system_status.b.serial_port_overflow_alarm) {
							system_status_msg.level = 2; // ERROR state
							system_status_msg.message = system_status_msg.message + "15. Data Output Overflow Alarm! ";
						}
						
						// Filter Status
						filter_status_msg.message = "";
						filter_status_msg.level = 0; // default OK state
						if (system_state_packet.filter_status.b.orientation_filter_initialised) {
							filter_status_msg.message = filter_status_msg.message + "0. Orientation Filter Initialised. ";	
						}
						else {
							filter_status_msg.level = 1; // WARN state
							filter_status_msg.message = filter_status_msg.message + "0. Orientation Filter NOT Initialised. ";
						}
						if (system_state_packet.filter_status.b.ins_filter_initialised) {
							filter_status_msg.message = filter_status_msg.message + "1. Navigation Filter Initialised. ";	
						}
						else {
							filter_status_msg.level = 1; // WARN state
							filter_status_msg.message = filter_status_msg.message + "1. Navigation Filter NOT Initialised. ";
						}
						if (system_state_packet.filter_status.b.heading_initialised) {
							filter_status_msg.message = filter_status_msg.message + "2. Heading Initialised. ";	
						}
						else {
							filter_status_msg.level = 1; // WARN state
							filter_status_msg.message = filter_status_msg.message + "2. Heading NOT Initialised. ";
						}
						if (system_state_packet.filter_status.b.utc_time_initialised) {
							filter_status_msg.message = filter_status_msg.message + "3. UTC Time Initialised. ";	
						}
						else {
							filter_status_msg.level = 1; // WARN state
							filter_status_msg.message = filter_status_msg.message + "3. UTC Time NOT Initialised. ";
						}											
						if (system_state_packet.filter_status.b.event1_flag) {
							filter_status_msg.level = 1; // WARN state
							filter_status_msg.message = filter_status_msg.message + "7. Event 1 Occured. ";	
						}
						else {							
							filter_status_msg.message = filter_status_msg.message + "7. Event 1 NOT Occured. ";
						}
						if (system_state_packet.filter_status.b.event2_flag) {
							filter_status_msg.level = 1; // WARN state
							filter_status_msg.message = filter_status_msg.message + "8. Event 2 Occured. ";	
						}
						else {
							filter_status_msg.message = filter_status_msg.message + "8. Event 2 NOT Occured. ";
						}
						if (system_state_packet.filter_status.b.internal_gnss_enabled) {
							filter_status_msg.message = filter_status_msg.message + "9. Internal GNSS Enabled. ";	
						}
						else {
							filter_status_msg.level = 1; // WARN state
							filter_status_msg.message = filter_status_msg.message + "9. Internal GNSS NOT Enabled. ";
						}
						if (system_state_packet.filter_status.b.magnetic_heading_enabled) {
							filter_status_msg.message = filter_status_msg.message + "10. Magnetic Heading Active. ";	
						}
						else {
							filter_status_msg.level = 1; // WARN state
							filter_status_msg.message = filter_status_msg.message + "10. Magnetic Heading NOT Active. ";
						}											
						if (system_state_packet.filter_status.b.velocity_heading_enabled) {							
							filter_status_msg.message = filter_status_msg.message + "11. Velocity Heading Enabled. ";	
						}
						else {							
							filter_status_msg.level = 1; // WARN state
							filter_status_msg.message = filter_status_msg.message + "11. Velocity Heading NOT Enabled. ";
						}
						if (system_state_packet.filter_status.b.atmospheric_altitude_enabled) {
							filter_status_msg.message = filter_status_msg.message + "12. Atmospheric Altitude Enabled. ";	
						}
						else {
							filter_status_msg.message = filter_status_msg.message + "12. Atmospheric Altitude NOT Enabled. ";
							filter_status_msg.level = 1; // WARN state
						}
						if (system_state_packet.filter_status.b.external_position_active) {
							filter_status_msg.message = filter_status_msg.message + "13. External Position Active. ";	
						}
						else {
							filter_status_msg.level = 1; // WARN state
							filter_status_msg.message = filter_status_msg.message + "13. External Position NOT Active. ";
						}
						if (system_state_packet.filter_status.b.external_velocity_active) {
							filter_status_msg.message = filter_status_msg.message + "14. External Velocity Active. ";	
						}
						else {
							filter_status_msg.level = 1; // WARN state
							filter_status_msg.message = filter_status_msg.message + "14. External Velocity NOT Active. ";
						}											
						if (system_state_packet.filter_status.b.external_heading_active) {
							filter_status_msg.message = filter_status_msg.message + "15. External Heading Active. ";	
						}
						else {							
							filter_status_msg.level = 1; // WARN state
							filter_status_msg.message = filter_status_msg.message + "15. External Heading NOT Active. ";
						}	
					}
				}
				
				// quaternion orientation standard deviation packet //
				if (an_packet->id == packet_id_quaternion_orientation_standard_deviation) 
				{
					// copy all the binary data into the typedef struct for the packet //
					// this allows easy access to all the different values             //
					if(decode_quaternion_orientation_standard_deviation_packet(&quaternion_orientation_standard_deviation_packet, an_packet) == 0)
					{	
						// IMU
						imu_msg.orientation_covariance[0] = quaternion_orientation_standard_deviation_packet.standard_deviation[0];
						imu_msg.orientation_covariance[4] = quaternion_orientation_standard_deviation_packet.standard_deviation[1];
						imu_msg.orientation_covariance[8] = quaternion_orientation_standard_deviation_packet.standard_deviation[2];						
					}
				}
				// Ensure that you free the an_packet when you're done with it //
				// or you will leak memory                                   //
				an_packet_free(&an_packet);

				// Publish messages //
				nav_sat_fix_pub.publish(nav_sat_fix_msg);
				twist_pub.publish(twist_msg);
				imu_pub.publish(imu_msg);
				system_status_pub.publish(system_status_msg);
				filter_status_pub.publish(filter_status_msg);
			}
		}
		ros::spinOnce();
	}

}
