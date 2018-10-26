#ifndef VOXBLOX_OCTOMAP_ROS_CONVERSIONS_TPP_
#define VOXBLOX_OCTOMAP_ROS_CONVERSIONS_TPP_

#include <voxblox_octomap_conversions/octomap_conversions.h>
#include <octomap_msgs/conversions.h>
#include <octomap/OcTree.h>


namespace voxblox {

    template<typename VoxelType>
    void serializeLayerAsOctomapMsg(const Layer <VoxelType> &layer,
                                    octomap_msgs::Octomap *msg, double surface_distance_threshold_factor) {
        octomap::OcTree tree(layer.voxel_size());
        serializeLayerAsOctomap(layer, &tree, surface_distance_threshold_factor);
        octomap_msgs::fullMapToMsg(tree, *msg);
    }

    template<typename VoxelType>
    void serializeLayerAsOctomap(const Layer <VoxelType> &layer,
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

        constexpr float kMinWeight = 1e-3;
        // Iterate over all blocks.
        for (const BlockIndex &index : blocks) {
            // Iterate over all voxels in said blocks.
            const Block<VoxelType> &block = layer.getBlockByIndex(index);

            for (size_t linear_index = 0; linear_index < num_voxels_per_block;
                 ++linear_index) {
                Point coord = block.computeCoordinatesFromLinearIndex(linear_index);
                const VoxelType &voxel = block.getVoxelByLinearIndex(linear_index);

                if (voxel.weight > kMinWeight &&
                    std::abs(voxel.distance) < surface_distance) {
                    octree->setNodeValue(coord.x(), coord.y(), coord.z(),octree->getClampingThresMaxLog());
                } /* else if(voxel.){
                    octree->setNodeValue(coord.x(), coord.y(), coord.z(),octree->getClampingThresMinLog());
                }*/

            }
        }
        octree->updateInnerOccupancy();
    }

}  // namespace voxblox

#endif  // VOXBLOX_OCTOMAP_ROS_CONVERSIONS_TPP_
