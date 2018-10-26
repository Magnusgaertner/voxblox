#ifndef VOXBLOX_OCTOMAP_ROS_CONVERSIONS_TPP_
#define VOXBLOX_OCTOMAP_ROS_CONVERSIONS_TPP_

#include <voxblox_octomap_conversions/octomap_conversions.h>
//#include <voxblox/core/layer.h>
//#include <voxblox_ros/ptcloud_vis.h>
//#include <voxblox/utils/evaluation_utils.h>
#include <octomap_msgs/conversions.h>
#include <octomap/OcTree.h>
namespace voxblox {

    template <typename VoxelType>
    void serializeLayerAsOctomapMsg(const Layer<VoxelType> &layer,
                                    octomap_msgs::Octomap *msg, double surface_distance_threshold_factor){
    octomap::OcTree tree(layer.voxel_size());
    serializeLayerAsOctomap(layer, &tree, surface_distance_threshold_factor);
    octomap_msgs::fullMapToMsg(tree, *msg);
  }


template <typename VoxelType>
void serializeLayerAsOctomap(const Layer<VoxelType> &layer,
                             octomap::OcTree *octree,
                             double surface_distance_threshold_factor) {

    octree->clear();
    octree->setResolution(layer.voxel_size());
    BlockIndexList blocks;
    layer.getAllAllocatedBlocks(&blocks);
    // Cache layer settings.
    size_t vps = layer.voxels_per_side();
    size_t num_voxels_per_block = vps * vps * vps;

    const FloatingPoint surface_distance = layer.voxel_size() * surface_distance_threshold_factor;

    // Iterate over all blocks.
    for (const BlockIndex &index : blocks) {
      // Iterate over all voxels in said blocks.
      const Block<VoxelType> &block = layer.getBlockByIndex(index);

      for (size_t linear_index = 0; linear_index < num_voxels_per_block;
           ++linear_index) {
        Point coord = block.computeCoordinatesFromLinearIndex(linear_index);
        const VoxelType& voxel = block.getVoxelByLinearIndex(linear_index);


        if (std::abs(voxel.distance) < surface_distance) {
            octree->updateNode(coord.x(), coord.y(), coord.z(), true);
        }/*else if(voxel.observed){
            octree->updateNode(coord.x(), coord.y(), coord.z(), false);
        }*/

      }
    }
  }




  template<typename VoxelType>
  bool deserializeOctomapMsgToLayer(const voxblox_msgs::Layer &msg,
                                    Layer<VoxelType> *layer) {
    /*
    CHECK_NOTNULL(layer);
    return deserializeMsgToLayer<VoxelType>(
        msg, static_cast<MapDerializationAction>(msg.action), layer);
        */
  }

  template<typename VoxelType>
  bool deserializeOctomapMsgToLayer(const voxblox_msgs::Layer &msg,
                                    const MapDerializationAction &action,
                                    Layer<VoxelType> *layer) {
    /*
    CHECK_NOTNULL(layer);
    if (getVoxelType<VoxelType>().compare(msg.layer_type) != 0) {
      return false;
    }

    // So we also need to check if the sizes match. If they don't, we can't
    // parse this at all.
    constexpr double kVoxelSizeEpsilon = 1e-5;
    if (msg.voxels_per_side != layer->voxels_per_side() ||
        std::abs(msg.voxel_size - layer->voxel_size()) > kVoxelSizeEpsilon) {
      return false;
    }

    if (action == MapDerializationAction::kReset) {
      layer->removeAllBlocks();
    }

    for (const voxblox_msgs::Block& block_msg : msg.blocks) {
      BlockIndex index(block_msg.x_index, block_msg.y_index, block_msg.z_index);

      // Either we want to update an existing block or there was no block there
      // before.
      if (action == MapDerializationAction::kUpdate || !layer->hasBlock(index)) {
        // Create a new block if it doesn't exist yet, or get the existing one
        // at the correct block index.
        typename Block<VoxelType>::Ptr block_ptr =
            layer->allocateBlockPtrByIndex(index);

        std::vector<uint32_t> data = block_msg.data;
        block_ptr->deserializeFromIntegers(data);

      } else if (action == MapDerializationAction::kMerge) {
        typename Block<VoxelType>::Ptr old_block_ptr =
            layer->getBlockPtrByIndex(index);
        CHECK(old_block_ptr);

        typename Block<VoxelType>::Ptr new_block_ptr(new Block<VoxelType>(
            old_block_ptr->voxels_per_side(), old_block_ptr->voxel_size(),
            old_block_ptr->origin()));

        std::vector<uint32_t> data = block_msg.data;
        new_block_ptr->deserializeFromIntegers(data);

        old_block_ptr->mergeBlock(*new_block_ptr);
      }
    }

    switch (action) {
      case MapDerializationAction::kReset:
        CHECK_EQ(layer->getNumberOfAllocatedBlocks(), msg.blocks.size());
        break;
      case MapDerializationAction::kUpdate:
        // Fall through intended.
      case MapDerializationAction::kMerge:
        CHECK_GE(layer->getNumberOfAllocatedBlocks(), msg.blocks.size());
        break;
    }

    return true;*/
  }

}  // namespace voxblox

#endif  // VOXBLOX_OCTOMAP_ROS_CONVERSIONS_TPP_
