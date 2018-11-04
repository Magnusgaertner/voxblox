#include <voxblox_ros/conversions.h>
#include <voxblox_ros/esdf_server.h>


#include "voxblox_ros/esdf_server.h"
#include "voxblox_ros/ros_params.h"
#include <swri_profiler/profiler.h>
namespace voxblox {

EsdfServer::EsdfServer(const ros::NodeHandle& nh,
                       const ros::NodeHandle& nh_private)
    : EsdfServer(nh, nh_private, getEsdfMapConfigFromRosParam(nh_private),
                 getEsdfIntegratorConfigFromRosParam(nh_private),
                 getTsdfMapConfigFromRosParam(nh_private),
                 getTsdfIntegratorConfigFromRosParam(nh_private)) {}

EsdfServer::EsdfServer(const ros::NodeHandle& nh,
                       const ros::NodeHandle& nh_private,
                       const EsdfMap::Config& esdf_config,
                       const EsdfIntegrator::Config& esdf_integrator_config,
                       const TsdfMap::Config& tsdf_config,
                       const TsdfIntegratorBase::Config& tsdf_integrator_config)
    : TsdfServer(nh, nh_private, tsdf_config, tsdf_integrator_config),
      clear_sphere_for_planning_(false),
      publish_esdf_map_(false),
      publish_traversable_(false),
      traversability_radius_(1.0),
      incremental_update_(true) {
  // Set up map and integrator.
  esdf_map_.reset(new EsdfMap(esdf_config));
  esdf_integrator_.reset(new EsdfIntegrator(esdf_integrator_config,
                                            tsdf_map_->getTsdfLayerPtr(),
                                            esdf_map_->getEsdfLayerPtr()));

  setupRos();
  inspectionSlider.reset(new voxblox::InteractiveSlider(nh_private_.resolveName("inspection"),this->getWorldFrame(),
                                                        [this](double level)
                                                        {
                                                          slice_level_ = level;

                                                          static ros::Time last_time(0);
                                                          ros::Time now = ros::Time::now();
                                                          if ((now - last_time).toSec() > 0.2) {

                                                            last_time = now;
                                                            this->publishSlices();
                                                          }
                                                        },Point(-2,0,0),
                                                        slice_axis_,0.5));
}

void EsdfServer::setupRos() {SWRI_PROFILE("setupRos");
  // Set up publisher.
  esdf_pointcloud_pub_ =
      nh_private_.advertise<pcl::PointCloud<pcl::PointXYZI> >("esdf_pointcloud",
                                                              1, true);
  esdf_slice_pub_ = nh_private_.advertise<pcl::PointCloud<pcl::PointXYZI> >(
      "esdf_slice", 1, true);
  traversable_pub_ = nh_private_.advertise<pcl::PointCloud<pcl::PointXYZI> >(
      "traversable", 1, true);

  esdf_map_pub_ =
      nh_private_.advertise<voxblox_msgs::Layer>("esdf_map_out", 1, false);

  // Set up subscriber.
  esdf_map_sub_ = nh_private_.subscribe("esdf_map_in", 4,
                                        &EsdfServer::esdfMapCallback, this);

  generate_esdf_srv_ = nh_private_.advertiseService(
      "generate_esdf", &EsdfServer::generateEsdfCallback, this);

  // Whether to clear each new pose as it comes in, and then set a sphere
  // around it to occupied.
  nh_private_.param("clear_sphere_for_planning", clear_sphere_for_planning_,
                    clear_sphere_for_planning_);
  nh_private_.param("publish_esdf_map", publish_esdf_map_, publish_esdf_map_);

  // Special output for traversible voxels. Publishes all voxels with distance
  // at least traversibility radius.
  nh_private_.param("publish_traversable", publish_traversable_,
                    publish_traversable_);
  nh_private_.param("traversability_radius", traversability_radius_,
                    traversability_radius_);

  init_esdf_update_policy();
}


    void EsdfServer::init_esdf_update_policy() {SWRI_PROFILE("init_esdf_update_policy");
      nh_private_.param("esdf_update_policy", update_policy, std::__cxx11::string("none"));
      nh_private_.param("update_esdf_every_n_scans", update_esdf_every_n_scans, 1);


      if(!update_policy.compare("timed")){
        double duration;
        nh_.param("esdf_update_rate", duration , 1.0);
        update_timer = nh_private_.createTimer(ros::Duration(1/duration), &EsdfServer::on_timed_esdf_update, this);
        update_timer.start();
      }
    }

void EsdfServer::on_timed_esdf_update(const ros::TimerEvent&){SWRI_PROFILE("on_timed_esdf_update");
  //ROS_ERROR("on timed esdf update called");
  updateEsdf();
}
void EsdfServer::publishAllUpdatedEsdfVoxels() {SWRI_PROFILE("publishAllUpdatedEsdfVoxels");
  // Create a pointcloud with distance = intensity.
  pcl::PointCloud<pcl::PointXYZI> pointcloud;

  createDistancePointcloudFromEsdfLayer(esdf_map_->getEsdfLayer(), &pointcloud);

  pointcloud.header.frame_id = world_frame_;
  esdf_pointcloud_pub_.publish(pointcloud);
}

void EsdfServer::publishSlices() {SWRI_PROFILE("publishSlices");
  TsdfServer::publishSlices();

  if(!esdf_slice_pub_.getNumSubscribers())return;
  pcl::PointCloud<pcl::PointXYZI> pointcloud;

  constexpr int kZAxisIndex = 2;
  createDistancePointcloudFromEsdfLayerSlice(
      esdf_map_->getEsdfLayer(), kZAxisIndex, slice_level_, &pointcloud);

  pointcloud.header.frame_id = world_frame_;
  pcl_conversions::toPCL(ros::Time::now(), pointcloud.header.stamp);
  esdf_slice_pub_.publish(pointcloud);
}

bool EsdfServer::generateEsdfCallback(
    std_srvs::Empty::Request& /*request*/,      // NOLINT
    std_srvs::Empty::Response& /*response*/) {SWRI_PROFILE("generateEsdfCallback");  // NOLINT
  const bool clear_esdf = true;
  if (clear_esdf) {
    esdf_integrator_->updateFromTsdfLayerBatch();
  } else {
    const bool clear_updated_flag = true;
    esdf_integrator_->updateFromTsdfLayer(clear_updated_flag);
  }
  publishAllUpdatedEsdfVoxels();
  publishSlices();
  return true;
}

void EsdfServer::updateMesh() {SWRI_PROFILE("updateMesh");
  // Also update the ESDF now, if there's any blocks in the TSDF.
  if (incremental_update_ &&
      tsdf_map_->getTsdfLayer().getNumberOfAllocatedBlocks() > 0) {
    const bool clear_updated_flag_esdf = false;
    esdf_integrator_->updateFromTsdfLayer(clear_updated_flag_esdf);
  }
  if (publish_pointclouds_) {
    publishAllUpdatedEsdfVoxels();
  }
  if (publish_traversable_) {
    publishTraversable();
  }
  if (publish_esdf_map_) {
    publishMap();
  }

  TsdfServer::updateMesh();
}

void EsdfServer::publishPointclouds() {SWRI_PROFILE("publishPointclouds");
  publishAllUpdatedEsdfVoxels();
  if (publish_slices_) {
    publishSlices();
  }

  if (publish_traversable_) {
    publishTraversable();
  }

  TsdfServer::publishPointclouds();
}

void EsdfServer::publishTraversable() {SWRI_PROFILE("publishTraversable");
  pcl::PointCloud<pcl::PointXYZI> pointcloud;
  createFreePointcloudFromEsdfLayer(esdf_map_->getEsdfLayer(),
                                    traversability_radius_, &pointcloud);
  pointcloud.header.frame_id = world_frame_;
  traversable_pub_.publish(pointcloud);
}

void EsdfServer::publishMap(const bool reset_remote_map) {SWRI_PROFILE("publishMap");
  if (this->esdf_map_pub_.getNumSubscribers() > 0) {
    const bool only_updated = false;
    timing::Timer publish_map_timer("map/publish_esdf");
    voxblox_msgs::Layer layer_msg;
    serializeLayerAsMsg<EsdfVoxel>(this->esdf_map_->getEsdfLayer(),
                                   only_updated, &layer_msg); //TODO this can be used to use diff only
    if (reset_remote_map) {
      layer_msg.action = static_cast<uint8_t>(MapDerializationAction::kReset);
    }
    this->esdf_map_pub_.publish(layer_msg);
    publish_map_timer.Stop();
  }

  TsdfServer::publishMap();
}

bool EsdfServer::saveMap(const std::string& file_path) {SWRI_PROFILE("saveMap");
  // Output TSDF map first, then ESDF.
  const bool success = TsdfServer::saveMap(file_path);

  constexpr bool kClearFile = false;
  return success &&
         io::SaveLayer(esdf_map_->getEsdfLayer(), file_path, kClearFile);
}

bool EsdfServer::loadMap(const std::string& file_path) {SWRI_PROFILE("loadMap");
  // Load in the same order: TSDF first, then ESDF.
  bool success = TsdfServer::loadMap(file_path);

  constexpr bool kMultipleLayerSupport = true;
  return success &&
         io::LoadBlocksFromFile(
             file_path, Layer<EsdfVoxel>::BlockMergingStrategy::kReplace,
             kMultipleLayerSupport, esdf_map_->getEsdfLayerPtr());
}

void EsdfServer::updateEsdf() {
  SWRI_PROFILE("updateEsdf");
  if (tsdf_map_->getTsdfLayer().getNumberOfAllocatedBlocks() > 0) {
    const bool clear_updated_flag_esdf = true;
    esdf_integrator_->updateFromTsdfLayer(clear_updated_flag_esdf);
  }
}

void EsdfServer::updateEsdfBatch(bool full_euclidean) {SWRI_PROFILE("updateEsdfBatch");
  if (tsdf_map_->getTsdfLayer().getNumberOfAllocatedBlocks() > 0) {
    esdf_integrator_->setFullEuclidean(full_euclidean);
    esdf_integrator_->updateFromTsdfLayerBatch();
  }
}

float EsdfServer::getEsdfMaxDistance() const {SWRI_PROFILE("getEsdfMaxDistance");
  return esdf_integrator_->getEsdfMaxDistance();
}

void EsdfServer::setEsdfMaxDistance(float max_distance) {SWRI_PROFILE("setEsdfMaxDistance");
  esdf_integrator_->setEsdfMaxDistance(max_distance);
}

float EsdfServer::getTraversabilityRadius() const {SWRI_PROFILE("getTraversabilityRadius");
  return traversability_radius_;
}

void EsdfServer::setTraversabilityRadius(float traversability_radius) {SWRI_PROFILE("setTraversabilityRadius");
  traversability_radius_ = traversability_radius;
}

void EsdfServer::newPoseCallback(const Transformation& T_G_C) {SWRI_PROFILE("newPoseCallback");
  if (clear_sphere_for_planning_) {
    esdf_integrator_->addNewRobotPosition(T_G_C.getPosition());
  }

  timing::Timer block_remove_timer("remove_distant_blocks");
  esdf_map_->getEsdfLayerPtr()->removeDistantBlocks(
      T_G_C.getPosition(), max_block_distance_from_body_);
  block_remove_timer.Stop();
}

void EsdfServer::esdfMapCallback(const voxblox_msgs::Layer& layer_msg) {SWRI_PROFILE("esdfMapCallback");
  bool success =
      deserializeMsgToLayer<EsdfVoxel>(layer_msg, esdf_map_->getEsdfLayerPtr());

  if (!success) {
    ROS_ERROR_THROTTLE(10, "Got an invalid ESDF map message!");
  } else {
    ROS_INFO("Got an ESDF map from ROS topic!");
    publishAllUpdatedEsdfVoxels();
    publishSlices();
  }
}

void EsdfServer::clear() {SWRI_PROFILE("clear");
  esdf_map_->getEsdfLayerPtr()->removeAllBlocks();
  esdf_integrator_->clear();
  CHECK_EQ(esdf_map_->getEsdfLayerPtr()->getNumberOfAllocatedBlocks(), 0u);

  TsdfServer::clear();

  // Publish a message to reset the map to all subscribers.
  constexpr bool kResetRemoteMap = true;
  publishMap(kResetRemoteMap);
}

    void EsdfServer::insertPointcloud(const sensor_msgs::PointCloud2::ConstPtr &pointcloud) {SWRI_PROFILE("insertPointcloud");
      TsdfServer::insertPointcloud(pointcloud);
      if(!update_policy.compare("on_integration")) {

        if(scan_num == update_esdf_every_n_scans){
          ROS_ERROR("updating esdf on integration");
          updateEsdf();
          scan_num = 0;
        }
        scan_num++;

      }
     // publishPointclouds();
    }

}  // namespace voxblox
