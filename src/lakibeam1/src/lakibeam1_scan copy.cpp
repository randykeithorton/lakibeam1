#include <rclcpp/rclcpp.hpp> 
#include <sensor_msgs/msg/laser_scan.hpp>

#include <stdio.h>
#include <pthread.h>
#include <unistd.h>
#include <sched.h>

#include <sys/select.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <math.h>
#include "../include/data_type.h"
#include "../include/remote.h"

#define DEG2RAD(x) ((x)*M_PI / 180.f)
using namespace std;
int main(int argc, char **argv)
{
	struct sockaddr_in ser_addr, clent_addr;
	std::string hostip, sensorip, port, frame_id, output_topic, scanfreq, filter, laser_enable, scan_range_start, scan_range_stop;
	std::vector <bm_response_scan_t> scan_vec;
	rclcpp::Time scan_begin, scan_end;
	bool inverted;
	int i = 0, j = 12;
	int angle_offset = 0;
	int resolution = 25;
	int scan_vec_ready = 0;
	int sockfd;
	rclcpp::Node::SharedPtr node_handle = nullptr;

	rclcpp::init(argc, argv);
	rclcpp::Time m_last_obstacle_msg_time {0, 0, RCL_ROS_TIME};

	node_handle = rclcpp::Node::make_shared("laser_scan_publisher");
  	// ros::NodeHandle nh("~");
	rclcpp::Rate rate(30); 
	frame_id = "laser";
	hostip = "0.0.0.0";
	sensorip = "192.168.198.2";
	port = "2368";
	output_topic = "scan";
	scanfreq = "30";
	filter = "3";
	laser_enable = "true";
	scan_range_start = "45";
	scan_range_stop = "315";
	angle_offset = 0;
	inverted = false;
	// node_handle->get_parameter<string>("frame_id",frame_id);
	// node_handle->get_parameter<std::string>("port",port);
	// node_handle->get_parameter<string>("hostip",hostip);
	// node_handle->get_parameter<string>("sensorip",sensorip);
	// node_handle->get_parameter<string>("output_topic",output_topic);
	// node_handle->get_parameter<string>("scanfreq",scanfreq);
	// node_handle->get_parameter<string>("filter",filter);
	// node_handle->get_parameter<string>("laser_enable",laser_enable);
	// node_handle->get_parameter<string>("scan_range_start",scan_range_start);
	// node_handle->get_parameter<string>("scan_range_stop",scan_range_stop);
	// node_handle->get_parameter<bool>("inverted",inverted);
	// node_handle->get_parameter<int>("angle_offset",angle_offset);
	// node_handle->declare_parameter<string>("frame_id",frame_id);
	// node_handle->declare_parameter<std::string>("port",port);
	// node_handle->declare_parameter<string>("hostip",hostip);
	// node_handle->declare_parameter<string>("sensorip",sensorip);
	// node_handle->declare_parameter<string>("output_topic",output_topic);
	// node_handle->declare_parameter<string>("scanfreq",scanfreq);
	// node_handle->declare_parameter<string>("filter",filter);
	// node_handle->declare_parameter<string>("laser_enable",laser_enable);
	// node_handle->declare_parameter<string>("scan_range_start",scan_range_start);
	// node_handle->declare_parameter<string>("scan_range_stop",scan_range_stop);
	// node_handle->declare_parameter<bool>("inverted",inverted);
	// node_handle->declare_parameter<int>("angle_offset",angle_offset);
	RCLCPP_INFO(node_handle->get_logger(),"frame_id:%s", frame_id.c_str());
	RCLCPP_INFO(node_handle->get_logger(),"output_topic:%s", output_topic.c_str());
	RCLCPP_INFO(node_handle->get_logger(),"inverted:%s", (inverted ? "True" : "False"));
	RCLCPP_INFO(node_handle->get_logger(),"hostip:%s", hostip.c_str());
	RCLCPP_INFO(node_handle->get_logger(),"sensorip:%s", sensorip.c_str());
	RCLCPP_INFO(node_handle->get_logger(),"port:%s", port.c_str());
	RCLCPP_INFO(node_handle->get_logger(),"scanfreq:%s", scanfreq.c_str());
	RCLCPP_INFO(node_handle->get_logger(),"filter:%s", filter.c_str());
	RCLCPP_INFO(node_handle->get_logger(),"laser_enable:%s", laser_enable.c_str());
	RCLCPP_INFO(node_handle->get_logger(),"scan_range_start:%s", scan_range_start.c_str());
	RCLCPP_INFO(node_handle->get_logger(),"scan_range_stop:%s", scan_range_stop.c_str());
	// ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan> (output_topic, 1000);
	rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub;
	scan_pub = node_handle->create_publisher<sensor_msgs::msg::LaserScan>(output_topic,1000);



	// sensor_config(sensorip, "/api/v1/sensor/scanfreq", scanfreq);
	// //sensor_config(sensorip, "/api/v1/sensor/filter", filter);
	// sensor_config(sensorip, "/api/v1/sensor/laser_enable", laser_enable);
	// sensor_config(sensorip, "/api/v1/sensor/scan_range/start", scan_range_start);
	// sensor_config(sensorip, "/api/v1/sensor/scan_range/stop", scan_range_stop);
	rclcpp::sleep_for(std::chrono::milliseconds(2000));
	RCLCPP_INFO(node_handle->get_logger(),"frame_id:%s", frame_id.c_str());

	// ros::Duration(2.0).sleep();
	// get_telemetry_data(sensorip);

	sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if(sockfd == -1)
	{
		RCLCPP_INFO(node_handle->get_logger(),"Failed to create socket");
		return -1;
    }

	memset(&ser_addr, 0, sizeof(ser_addr));
    ser_addr.sin_family = AF_INET;
	ser_addr.sin_addr.s_addr = inet_addr(hostip.c_str());
    ser_addr.sin_port = htons(atoi(port.c_str()));

    if(bind(sockfd, (struct sockaddr*)&ser_addr, sizeof(ser_addr)) < 0)
	{
		RCLCPP_INFO(node_handle->get_logger(),"Socket bind error!");
		return -1;
	}

	while (rclcpp::ok())
	{
		if(scan_vec_ready == 0)
		{
			while(1)
			{
				if(j == 12)
				{
					unsigned int len = sizeof(clent_addr);
					recvfrom(sockfd, &MSOP_Data, sizeof(MSOP_Data), 0, (struct sockaddr*)&clent_addr, &len);
					if(MSOP_Data.BlockID[0].Azimuth == 0)
					{
						scan_end = scan_begin;
						scan_begin = rclcpp::Clock().now();
					}			
					if((MSOP_Data.BlockID[1].Azimuth - MSOP_Data.BlockID[0].Azimuth) > 0)
					{
						resolution = (MSOP_Data.BlockID[1].Azimuth - MSOP_Data.BlockID[0].Azimuth) / 16;
					}
					j = 0;
				}

				for(;j < 12; j++)
				{
					for(i = 0; i < 16; i++)
					{
						bm_response_scan_t response_ptr;
						response_ptr.angle = (MSOP_Data.BlockID[j].Azimuth + (resolution * i));
						if(MSOP_Data.BlockID[j].DataFlag == 0xEEFF)
						{
							if(response_ptr.angle == 0)
							{
								if(!scan_vec.empty() & (scan_vec_ready == 0))
								{
									scan_vec_ready = 1;
									if(scan_vec.size() < 1200)
									{
										j = 12;
									}
									break;
								}
							}
							response_ptr.dist = MSOP_Data.BlockID[j].Result[i].Dist_1;
							response_ptr.rssi = MSOP_Data.BlockID[j].Result[i].RSSI_1;
							scan_vec.push_back(response_ptr);
						}
					}
					if(scan_vec_ready == 1)
					{
						break;
					}
				}
				if(scan_vec_ready == 1)
				{
					break;
				}
			}
		}

		if(scan_vec_ready == 1)
		{
			sensor_msgs::msg::LaserScan scan;
			uint16_t num_readings;
			float duration = (scan_begin - scan_end).seconds();

			num_readings = scan_vec.size();
			scan.header.stamp = scan_begin;
			scan.header.frame_id = frame_id;
			scan.angle_min = DEG2RAD(-180 + angle_offset);
			scan.angle_max = DEG2RAD(180 + angle_offset);
			scan.angle_increment = 2.0 * M_PI / num_readings;
			scan.scan_time = duration;
			scan.time_increment = duration/(float)num_readings;
			scan.range_min = 0.0;
			scan.range_max = 100.0;
			scan.ranges.resize(num_readings);
			scan.intensities.resize(num_readings);

			for(int i = 0;i < num_readings; i++)
			{
				if (!inverted)
    			{
					scan.ranges[i] = (float)scan_vec[i].dist / 1000;
					scan.intensities[i] = scan_vec[i].rssi;
				}
				else
				{
					scan.ranges[num_readings - i - 1] = (float)scan_vec[i].dist / 1000;
					scan.intensities[num_readings - i - 1] = scan_vec[i].rssi;
				}
			}

			scan_pub->publish(scan);
			RCLCPP_INFO(node_handle->get_logger(),"New topic %s published, total data points: %d", output_topic.c_str(), num_readings);
			scan_vec.clear();
			scan_vec_ready = 0;
		}
	}
	close(sockfd);

	return 0;
}