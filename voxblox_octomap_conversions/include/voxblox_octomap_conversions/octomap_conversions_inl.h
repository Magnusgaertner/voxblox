#ifndef VOXBLOX_OCTOMAP_ROS_CONVERSIONS_TPP_
#define VOXBLOX_OCTOMAP_ROS_CONVERSIONS_TPP_

#include <voxblox_octomap_conversions/octomap_conversions.h>
#include <octomap_msgs/conversions.h>
#include <octomap/OcTree.h>
#define SWRI_ENABLED


#ifdef SWRI_ENABLED
    #include <swri_profiler/profiler.h>
#endif

namespace voxblox {

    template<typename VoxelType>
    void serializeLayerAsOctomapMsg(const Layer <VoxelType> &layer,
                                    octomap_msgs::Octomap *msg, double surface_distance_threshold_factor) {
        octomap::OcTree tree(layer.voxel_size());
        {
            #ifdef SWRI_ENABLED
                SWRI_PROFILE("serializeLayerAsOctomap");
            #endif
            serializeLayerAsOctomap(layer, &tree, surface_distance_threshold_factor);
        }
        {
            #ifdef SWRI_ENABLED
                SWRI_PROFILE("fullMapToMsg");
            #endif
            octomap_msgs::fullMapToMsg(tree, *msg);
        }
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

                {
                    #ifdef SWRI_ENABLED
                        SWRI_PROFILE("tsdf_get_voxel_coordinates_and_value_by_linear_index");
                    #endif
                    const auto& coord = block.computeCoordinatesFromLinearIndex(linear_index);
                    const auto& voxel = block.getVoxelByLinearIndex(linear_index);
                    if (voxel.weight > kMinWeight &&
                        std::abs(voxel.distance) < surface_distance) {
                        {
                            #ifdef SWRI_ENABLED
                                SWRI_PROFILE("insert_into_octree");
                            #endif
                            octree->setNodeValue(coord.x(), coord.y(), coord.z(), octree->getClampingThresMaxLog());
                        }
                    } /* else if(voxel.){
                    octree->setNodeValue(coord.x(), coord.y(), coord.z(),octree->getClampingThresMinLog());
                }*/
                }


            }
        }
        {
            #ifdef SWRI_ENABLED
                SWRI_PROFILE("updateInnerOccupancy");
            #endif
            octree->updateInnerOccupancy();
        }
    }

    template <typename T>
    void SerializeToByteArray_(const T& msg, std::vector<uint8_t>& destination_buffer)
    {
        const uint32_t length = ros::serialization::serializationLength(msg);
        destination_buffer.resize( length );
        //copy into your own buffer
        ros::serialization::OStream stream(destination_buffer.data(), length);
        ros::serialization::serialize(stream, msg);
    }
/*
    void wrapLayerMsgToOctomapMsg(const voxblox_msgs::Layer &layer, const octomap_msgs::Octomap* msg){

    }
    void unwrapLayerMsgFromOctomapMsg(voxblox_msgs::Layer* layer, octomap_msgs::Octomap& msg){

    }*/



}  // namespace voxblox

#endif  // VOXBLOX_OCTOMAP_ROS_CONVERSIONS_TPP_
