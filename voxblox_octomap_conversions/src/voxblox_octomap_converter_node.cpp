//
// Created by magnus on 25.10.18.
//

#include <voxblox_octomap_conversions/octomap_conversions.h>
#include <ros/ros.h>
#include <voxblox_ros/esdf_server.h>
int main(){

  voxblox::EsdfServer server(ros::NodeHandle(""), ros::NodeHandle(""));
  octomap_msgs::Octomap octomap_msg;

  voxblox::serializeLayerAsOctomapMsg(  server.getEsdfMapPtr()->getEsdfLayer(), &octomap_msg,1);
}
