#ifndef VOXBLOX_OCTOMAP_ROS_CONVERSIONS_H_
#define VOXBLOX_OCTOMAP_ROS_CONVERSIONS_H_



#include <algorithm>
#include <vector>

#include <voxblox/core/common.h>
#include <voxblox/core/layer.h>
#include <voxblox_msgs/Layer.h>
#include <voxblox_ros/conversions.h>

#include <octomap/OcTree.h>
#include <octomap_msgs/Octomap.h>

namespace voxblox {


// Declarations
  template <typename VoxelType>
  void serializeLayerAsOctomapMsg(const Layer<VoxelType> &layer,
                                  octomap_msgs::Octomap *msg, double surface_distance_threshold_factor);


  template <typename VoxelType>
  void serializeLayerAsOctomap(const Layer<VoxelType> &layer,
                               octomap::OcTree *octree,
                               double surface_distance_threshold_factor);


  template<typename  VoxelType>
  bool default_is_occupied_strategy(const FloatingPoint surface_distance, const VoxelType &voxel);
// Returns true if could parse the data into the existing layer (all parameters
// are compatible), false otherwise.
// This function will use the deserialization action suggested by the layer
// message.
  template <typename VoxelType>
  bool deserializeOctomapMsgToLayer(const voxblox_msgs::Layer& msg,
                             Layer<VoxelType>* layer);

  template <typename VoxelType>
  bool deserializeOctomapMsgToLayer(const voxblox_msgs::Layer& msg,
                             const MapDerializationAction& action,
                             Layer<VoxelType>* layer);

}  // namespace voxblox

#endif  // VOXBLOX_OCTOMAP_ROS_CONVERSIONS_H_

#include <voxblox_octomap_conversions/octomap_conversions_inl.h>
