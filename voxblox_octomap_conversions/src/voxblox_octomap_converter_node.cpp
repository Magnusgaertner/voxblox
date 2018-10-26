//
// Created by magnus on 25.10.18.
//

#include <voxblox_octomap_conversions/octomap_conversions.h>
#include <voxblox_ros/esdf_server.h>
#include "ros/ros.h"
#include "std_msgs/String.h"

class Converter{
public:
    Converter(ros::NodeHandle& n){
      pub = n.advertise<octomap_msgs::Octomap>("octomap_out",1,true);
    }
    ros::Publisher pub;
    void conversionCB(const voxblox_msgs::LayerConstPtr& layer_msg)
    {
      octomap::OcTree octomap(layer_msg->voxel_size);
      voxblox::Layer<voxblox::TsdfVoxel> layer(layer_msg->voxel_size, layer_msg->voxels_per_side);
      voxblox::deserializeMsgToLayer<voxblox::TsdfVoxel>(*layer_msg, &layer);
      voxblox::serializeLayerAsOctomap<voxblox::TsdfVoxel>(layer,&octomap,0.75);
      octomap_msgs::Octomap octomap_msg;
      //octomap_msgs::binaryMapToMsg(octomap, octomap_msg);

      octomap_msg.header.frame_id = "base_footprint";
      octomap_msg.header.stamp = ros::Time::now();
      octomap_msg.id = "OcTree";
      pub.publish(octomap_msg);
    }
};


int main(int argc, char **argv)
{

  ros::init(argc, argv, "voxblox_tsdf_to_octomap_converter");
  ros::NodeHandle n("~");
  Converter c(n);
  ros::Subscriber sub = n.subscribe("layer_in", 1, &Converter::conversionCB, &c);
  ros::spin();
  return 0;
}
