#include <ros/ros.h>
#include <range_msgs/P2PRange.h>
#include <range_msgs/P2PRangeWithPose.h>
#include <string>
#include "idolink_dev.hpp"

struct Anchor
{
	uint64_t id;
	std::string frame_id;
	double x;
	double y;
	double z;
};

void loadAnchorList(std::vector<Anchor> &list)
{
	ros::NodeHandle lnh("~");
	
	// Clear anchors list
	list.clear();
	
	// Get the number of anchors to read
	int N;
	if(!lnh.getParam("anchor_list/N", N))
		N = 0;
	
	// Read each anchor and store in the list
	Anchor data;
	std::string id;
	for(int i=0; i<N; i++)
	{
		std::string s = "anchor_list/a" + boost::lexical_cast<std::string>(i) + "/";
		if(lnh.getParam(s+"id", id) && 
		   lnh.getParam(s+"frame_id", data.frame_id) && 
		   lnh.getParam(s+"x", data.x) && 
		   lnh.getParam(s+"y", data.y) && 
		   lnh.getParam(s+"z", data.z))
		{
			if(id.find("0x") == 0 || id.find("0X") == 0)
				sscanf(id.c_str(), "%lx", &data.id);
			else
				sscanf(id.c_str(), "%lu", &data.id);
			list.push_back(data);
		}
	}	
}

int main(int argc, char** argv) 
{
	IdolinkDev node;
	IdolinkDev::RangeData data;
	ros::init(argc, argv, "idolink_node"); 
	ros::NodeHandle nh; 
	ros::NodeHandle lnh("~");
	
	// Read node parameters
	bool useBinProtocol = false;
	std::string device, frame_id, protocol;
	std::vector<Anchor> anchors;
	if(!lnh.getParam("device", device))
	{
		std::cout << "[idolink] No device specified! Assumming '/dev/ttyUSB0' as default" << std::endl;
		device = "/dev/ttyUSB0";	
	}
	if(!lnh.getParam("frame_id", frame_id))
		frame_id = "/idolink";	
	if(!lnh.getParam("protocol", protocol))
		protocol = "bin";	
	if(protocol != "bin" && protocol != "char")
	{
		std::cout << "[idolink] Unknown serial protocol. Assumming 'bin' as default" << std::endl;
		protocol = "bin";	
	}
	if(protocol == "bin")
		useBinProtocol = true;
	loadAnchorList(anchors);

	// Initialize serial device
	if(!node.init(device, useBinProtocol))
	{
		std::cout << "[idolink] Error opening serial device " << device << std::endl;
		return -1;
	}
	
	// Advertise publishers
	ros::Publisher pubRange = nh.advertise<range_msgs::P2PRange>("/idolink_node/range", 1);
	ros::Publisher pubAnchorRange = nh.advertise<range_msgs::P2PRangeWithPose>("/idolink_node/anchor/range", 1);
	range_msgs::P2PRange rangeMsg;
	range_msgs::P2PRangeWithPose anchorRangeMsg;
	
	// Read range messages and publish
	while(ros::ok())
	{
		// Wait until new range is received
		if(node.readRange(data) != 0)
			continue;
		//printf("[idolink] Range %lx --> %lx: %f m, RSSI:%f dBm\n", data.sourceId, data.destinationId, data.range, data.rssi);
		
		// Fill-up msg and publish
		rangeMsg.header.seq = data.seq;
		rangeMsg.header.frame_id = frame_id;
		rangeMsg.header.stamp = ros::Time::now();
		rangeMsg.radiation_type = range_msgs::P2PRange::RADIO;
		rangeMsg.source_id = data.sourceId;
		rangeMsg.source_type = range_msgs::P2PRange::TAG;
		rangeMsg.destination_id = data.destinationId;
		rangeMsg.destination_type = range_msgs::P2PRange::BASE;
		rangeMsg.rssi = data.rssi;
		rangeMsg.range = data.range;
		rangeMsg.variance = 0.2*0.2;
		pubRange.publish(rangeMsg);
		
		// Publish anchor range if destination was an anchor
		for(int i=0; i<anchors.size(); i++)
		{
			if(anchors[i].id == data.sourceId)
			{
				anchorRangeMsg.header = rangeMsg.header;
				anchorRangeMsg.radiation_type = range_msgs::P2PRange::RADIO;
				anchorRangeMsg.source_id = data.sourceId;
				anchorRangeMsg.source_type = range_msgs::P2PRange::ANCHOR;
				anchorRangeMsg.destination_id = data.destinationId;
				anchorRangeMsg.destination_type = range_msgs::P2PRange::BASE;
				anchorRangeMsg.rssi = data.rssi;
				anchorRangeMsg.range = data.range;
				anchorRangeMsg.variance = 0.2*0.2;
				anchorRangeMsg.position.header = rangeMsg.header;
				anchorRangeMsg.position.header.frame_id = anchors[i].frame_id;
				anchorRangeMsg.position.point.x = anchors[i].x;
				anchorRangeMsg.position.point.y = anchors[i].y;
				anchorRangeMsg.position.point.z = anchors[i].z;
				pubAnchorRange.publish(anchorRangeMsg);
				
				break;
			}
		}
		
		// Spin
		ros::spinOnce();
	}
	
	// Close serial comms
	node.finish();

	return 0;
}

