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

#include <ros/ros.h>
#include <tf/tf.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <unistd.h>

#include <diagnostic_msgs/DiagnosticArray.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/TimeReference.h>

#include "rs232/rs232.h"
#include "an_packet_protocol.h"
#include "spatial_packets.h"


#define RADIANS_TO_DEGREES (180.0/M_PI)

// Convert NED orientation (Advanced Navigation) to ENU orientation (ROS)
inline geometry_msgs::Quaternion nedToEnu(geometry_msgs::Quaternion const & in) {
	tf::Quaternion enu(in.x, in.y, in.z, in.w); // x, y, z, w according to tf documentation
	// To get from NED to ENU we have to:
	// 1) Rotate 90 degrees around Z
	// 2) Rotate the result 180 degrees around Y
	tf::Transform z_90(tf::Quaternion(tf::Vector3(0, 0, 1), 0.5*M_PI));
	tf::Transform y_180(tf::Quaternion(tf::Vector3(0, 1, 0), M_PI));

	enu = (y_180 * (z_90 * enu));
	geometry_msgs::Quaternion out;
	out.x = enu.x();
	out.y = enu.y();
	out.z = enu.z();
	out.w = enu.w();
	return out;
}

// Convert the packet status to a ros diagnostic message
inline void appendSystemStatus(diagnostic_msgs::DiagnosticArray & status_array, unsigned int const status, int const level, std::string const & failure_message = "", std::string const & success_message = "") {
	if (status) {
		diagnostic_msgs::DiagnosticStatus status_msg;
		status_msg.level = level; // WARN=1 ERROR=2
		status_msg.message = failure_message;
		status_array.status.push_back(status_msg);
	}
}


// Convert the packet status to a ros diagnostic message
inline diagnostic_msgs::DiagnosticStatus filterStatusToMsg(unsigned int const status, int const level, std::string const & failure_message = "", std::string const & success_message = "") {
	diagnostic_msgs::DiagnosticStatus status_msg;
	status_msg.message = success_message;
	status_msg.level = 0; // default OK state
	if (!status) {
		status_msg.level = level; // WARN=1 ERROR=2
		status_msg.message = failure_message;
	}
	return status_msg;
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

	// Initialise Publishers and Topics //
	ros::Publisher nav_sat_fix_pub=nh.advertise<sensor_msgs::NavSatFix>("nav_sat_fix",10);
	ros::Publisher imu_pub=nh.advertise<sensor_msgs::Imu>("imu",10);
	ros::Publisher odom_pub=nh.advertise<nav_msgs::Odometry>("odom",10);
	ros::Publisher diagnostics_pub=nh.advertise<diagnostic_msgs::DiagnosticArray>("diagnostics",10);

	// Initialise gps message
	sensor_msgs::NavSatFix nav_sat_fix_msg;
	nav_sat_fix_msg.header.frame_id="an_device";
	nav_sat_fix_msg.position_covariance_type=2; // fixed to variance on the diagonal
	nav_sat_fix_msg.position_covariance = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

	// Initialize wheel odometry message
	nav_msgs::Odometry odom_msg;
	odom_msg.header.frame_id = "odom";
	odom_msg.child_frame_id = "an_device";
	odom_msg.twist.covariance = {
		1.0, 0  , 0  , 0, 0, 0,
		0  , 1.0, 0  , 0, 0, 0,
		0  , 0  , 1.0, 0, 0, 0,
		0  , 0  , 0  , 0, 0, 0,
		0  , 0  , 0  , 0, 0, 0,
		0  , 0  , 0  , 0, 0, 0
	};

	// Initialize imu message
	sensor_msgs::Imu imu_msg;
	imu_msg.header.frame_id = "an_device";
	imu_msg.orientation_covariance = { // Will be read from device
		0.0, 0.0, 0.0,
		0.0, 0.0, 0.0,
		0.0, 0.0, 0.0};
	imu_msg.angular_velocity_covariance = { // Cannot be read from device
		0.2, 0.0, 0.0,
		0.0, 0.2, 0.0,
		0.0, 0.0, 0.2};
	imu_msg.linear_acceleration_covariance = { // Cannot be read from device
		1.0, 0.0, 0.0,
		0.0, 1.0, 0.0,
		0.0, 0.0, 1.0};

	// Status messages
	diagnostic_msgs::DiagnosticArray diagnostics_msg;

	// get data from com port //
	an_decoder_t an_decoder;
	an_packet_t *an_packet;

	// packet types
	system_state_packet_t                                system_state_packet;                          // Packet 20
	euler_orientation_standard_deviation_packet_t        euler_orientation_standard_deviation_packet;  // Packet 26
	body_velocity_packet_t                               body_velocity_packet;                         // Packet 36
	quaternion_orientation_packet_t                      quaternion_orientation_packet;                // Packet 40

	if (OpenComport(com_port, baud_rate))
	{
		printf("Could not open serial port: %s \n",com_port);
		exit(EXIT_FAILURE);
	}

	an_decoder_initialise(&an_decoder);
	int bytes_received;

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

				// acknowledgement packet (0) //
				if (an_packet->id == 0)
				{
					ROS_INFO("acknowledgement data: %d", an_packet->data[3]);
				}


				// system state packet (20) //
				if (an_packet->id == packet_id_system_state)
				{
					if(decode_system_state_packet(&system_state_packet, an_packet) == 0)
					{
						// NavSatFix
						nav_sat_fix_msg.header.stamp.sec=system_state_packet.unix_time_seconds;
						nav_sat_fix_msg.header.stamp.nsec=system_state_packet.microseconds*1000;
						if ((system_state_packet.filter_status.b.gnss_fix_type == 1) ||
							(system_state_packet.filter_status.b.gnss_fix_type == 2))
						{
							nav_sat_fix_msg.status.status=0;
						}
						else if ((system_state_packet.filter_status.b.gnss_fix_type == 3) ||
							 (system_state_packet.filter_status.b.gnss_fix_type == 5))
						{
							nav_sat_fix_msg.status.status=1;
						}
						else if ((system_state_packet.filter_status.b.gnss_fix_type == 4) ||
							 (system_state_packet.filter_status.b.gnss_fix_type == 6) ||
							 (system_state_packet.filter_status.b.gnss_fix_type == 7))
						{
							nav_sat_fix_msg.status.status=2;
						}
						else
						{
							nav_sat_fix_msg.status.status=-1;
						}
						nav_sat_fix_msg.latitude=system_state_packet.latitude * RADIANS_TO_DEGREES;
						nav_sat_fix_msg.longitude=system_state_packet.longitude * RADIANS_TO_DEGREES;
						nav_sat_fix_msg.altitude=system_state_packet.height;
						nav_sat_fix_msg.position_covariance[0] = pow(system_state_packet.standard_deviation[1],2);
						nav_sat_fix_msg.position_covariance[4] = pow(system_state_packet.standard_deviation[0],2);
						nav_sat_fix_msg.position_covariance[8] = pow(system_state_packet.standard_deviation[2],2);

						// IMU
						imu_msg.header.stamp.sec=system_state_packet.unix_time_seconds;
						imu_msg.header.stamp.nsec=system_state_packet.microseconds*1000;
						imu_msg.linear_acceleration.x = system_state_packet.body_acceleration[0];
						imu_msg.linear_acceleration.y = system_state_packet.body_acceleration[1];
						imu_msg.linear_acceleration.z = system_state_packet.body_acceleration[2];
						imu_msg.angular_velocity.x = system_state_packet.angular_velocity[0];
						imu_msg.angular_velocity.y = system_state_packet.angular_velocity[1];
						imu_msg.angular_velocity.z = system_state_packet.angular_velocity[2];

						// Filter Status
						diagnostics_msg.header.stamp.sec=system_state_packet.unix_time_seconds;
						diagnostics_msg.header.stamp.nsec=system_state_packet.microseconds*1000;
						diagnostics_msg.status.clear();
						diagnostics_msg.status.push_back(filterStatusToMsg(system_state_packet.filter_status.b.orientation_filter_initialised, 1, "Orientation Filter NOT Initialised", "Orientation Filter Initialised"));
						diagnostics_msg.status.push_back(filterStatusToMsg(system_state_packet.filter_status.b.ins_filter_initialised        , 1, "Navigation Filter NOT Initialised", "Navigation Filter Initialised"));
						diagnostics_msg.status.push_back(filterStatusToMsg(system_state_packet.filter_status.b.heading_initialised           , 1, "Heading NOT Initialised", "Heading Initialised"));
						diagnostics_msg.status.push_back(filterStatusToMsg(system_state_packet.filter_status.b.utc_time_initialised          , 1, "UTC Time NOT Initialised", "UTC Time Initialised"));
						diagnostics_msg.status.push_back(filterStatusToMsg(system_state_packet.filter_status.b.event1_flag                   , 1, "Event 1 NOT Occurred", "Event 1 Occurred"));
						diagnostics_msg.status.push_back(filterStatusToMsg(system_state_packet.filter_status.b.event2_flag                   , 1, "Event 2 NOT Occurred", "Event 2 Occurred"));
						diagnostics_msg.status.push_back(filterStatusToMsg(system_state_packet.filter_status.b.internal_gnss_enabled         , 1, "Internal GNSS NOT Enabled", "Internal GNSS Enabled"));
						diagnostics_msg.status.push_back(filterStatusToMsg(system_state_packet.filter_status.b.magnetic_heading_enabled      , 1, "Magnetic Heading NOT Active", "Magnetic Heading Active"));
						diagnostics_msg.status.push_back(filterStatusToMsg(system_state_packet.filter_status.b.velocity_heading_enabled      , 1, "Velocity Heading NOT Enabled", "Velocity Heading Enabled"));
						diagnostics_msg.status.push_back(filterStatusToMsg(system_state_packet.filter_status.b.atmospheric_altitude_enabled  , 1, "Atmospheric Altitude NOT Enabled", "Atmospheric Altitude Enabled"));
						diagnostics_msg.status.push_back(filterStatusToMsg(system_state_packet.filter_status.b.external_position_active      , 1, "External Position NOT Active", "External Position Active"));
						diagnostics_msg.status.push_back(filterStatusToMsg(system_state_packet.filter_status.b.external_velocity_active      , 1, "External Velocity NOT Active", "External Velocity Active"));
						diagnostics_msg.status.push_back(filterStatusToMsg(system_state_packet.filter_status.b.external_heading_active       , 1, "External Heading NOT Active", "External Heading Active"));

						// System Status
						appendSystemStatus(diagnostics_msg, system_state_packet.system_status.b.system_failure              , 2, "System Failure!");
						appendSystemStatus(diagnostics_msg, system_state_packet.system_status.b.system_failure              , 2, "System Failure!");
						appendSystemStatus(diagnostics_msg, system_state_packet.system_status.b.accelerometer_sensor_failure, 2, "Accelerometer Sensor Failure!");
						appendSystemStatus(diagnostics_msg, system_state_packet.system_status.b.gyroscope_sensor_failure    , 2, "Gyroscope Sensor Failure!");
						appendSystemStatus(diagnostics_msg, system_state_packet.system_status.b.magnetometer_sensor_failure , 2, "Magnetometer Sensor Failure!");
						appendSystemStatus(diagnostics_msg, system_state_packet.system_status.b.pressure_sensor_failure     , 2, "Pressure Sensor Failure!");
						appendSystemStatus(diagnostics_msg, system_state_packet.system_status.b.gnss_failure                , 2, "GNSS Failure!");
						appendSystemStatus(diagnostics_msg, system_state_packet.system_status.b.accelerometer_over_range    , 2, "Accelerometer Over Range!");
						appendSystemStatus(diagnostics_msg, system_state_packet.system_status.b.gyroscope_over_range        , 2, "Gyroscope Over Range!");
						appendSystemStatus(diagnostics_msg, system_state_packet.system_status.b.magnetometer_over_range     , 2, "Magnetometer Over Range!");
						appendSystemStatus(diagnostics_msg, system_state_packet.system_status.b.pressure_over_range         , 2, "Pressure Over Range!");
						appendSystemStatus(diagnostics_msg, system_state_packet.system_status.b.minimum_temperature_alarm   , 2, "Minimum Temperature Alarm!");
						appendSystemStatus(diagnostics_msg, system_state_packet.system_status.b.maximum_temperature_alarm   , 2, "Maximum Temperature Alarm!");
						appendSystemStatus(diagnostics_msg, system_state_packet.system_status.b.low_voltage_alarm           , 2, "Low Voltage Alarm!");
						appendSystemStatus(diagnostics_msg, system_state_packet.system_status.b.high_voltage_alarm          , 2, "High Voltage Alarm!");
						appendSystemStatus(diagnostics_msg, system_state_packet.system_status.b.gnss_antenna_disconnected   , 2, "GNSS Antenna Disconnected!");
						appendSystemStatus(diagnostics_msg, system_state_packet.system_status.b.serial_port_overflow_alarm  , 2, "Data Output Overflow Alarm!");

					}
				}

				// euler orientation standard deviation packet (26) //
				if (an_packet->id == packet_id_euler_orientation_standard_deviation)
				{
					if(decode_euler_orientation_standard_deviation_packet(&euler_orientation_standard_deviation_packet, an_packet) == 0)
					{
						imu_msg.orientation_covariance[0] = pow(euler_orientation_standard_deviation_packet.standard_deviation[0],2);
						imu_msg.orientation_covariance[4] = pow(euler_orientation_standard_deviation_packet.standard_deviation[1],2);
						imu_msg.orientation_covariance[8] = pow(euler_orientation_standard_deviation_packet.standard_deviation[2],2);
					}
				}

				// body velocity packet (36)
				if (an_packet->id == packet_id_body_velocity) {
					if(decode_body_velocity_packet(&body_velocity_packet, an_packet) == 0) {
						odom_msg.header.stamp.sec=system_state_packet.unix_time_seconds;
						odom_msg.header.stamp.nsec=system_state_packet.microseconds*1000;
						odom_msg.twist.twist.linear.x = body_velocity_packet.velocity[0];
						odom_msg.twist.twist.linear.y = body_velocity_packet.velocity[1]; // Should be close to zero
						odom_msg.twist.twist.linear.z = body_velocity_packet.velocity[2]; // Should be close to zero
					}
				}


				// quaternion orientation packet (40) //
				if (an_packet->id == packet_id_quaternion_orientation)
				{
					if(decode_quaternion_orientation_packet(&quaternion_orientation_packet, an_packet) == 0)
					{
						imu_msg.orientation.x = quaternion_orientation_packet.orientation[1];    // AN reports in s (w?), x, y, z
						imu_msg.orientation.y = quaternion_orientation_packet.orientation[2];
						imu_msg.orientation.z = quaternion_orientation_packet.orientation[3];
						imu_msg.orientation.w = quaternion_orientation_packet.orientation[0];
						imu_msg.orientation   = nedToEnu(imu_msg.orientation);                   // AN reports in NED, ROS expects ENU
					}
				}



				// receiver information packet (69) //
				if (an_packet->id == 69)
				{
					ROS_INFO("receiver information: %d", an_packet->data[0]);
				}


				// Ensure that you free the an_packet when your done with it //
				// or you will leak memory                                   //
				an_packet_free(&an_packet);

				// Publish messages //
				nav_sat_fix_pub.publish(nav_sat_fix_msg);
				odom_pub.publish(odom_msg);
				imu_pub.publish(imu_msg);
				diagnostics_pub.publish(diagnostics_msg);
			}
		}
	}

}

