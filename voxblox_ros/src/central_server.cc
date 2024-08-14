#include <voxblox_ros/central_server.h>

#include "voxblox_ros/conversions.h"
#include "voxblox_ros/ros_params.h"

namespace voxblox {

CentralServer::CentralServer(const ros::NodeHandle& nh,
                             const ros::NodeHandle& nh_private) 
                             : CentralServer(nh, nh_private,
                             getEsdfMapConfigFromRosParam(nh_private),
                             getTsdfMapConfigFromRosParam(nh_private),
                             getMeshIntegratorConfigFromRosParam(nh_private)) {}

CentralServer::CentralServer(const ros::NodeHandle& nh,
                       const ros::NodeHandle& nh_private,
                       const EsdfMap::Config& esdf_config,
                       const TsdfMap::Config& tsdf_config,
                       const MeshIntegratorConfig& mesh_config)
    : nh_(nh),
      nh_private_(nh_private),
      verbose_(true),
      world_frame_("world"),
      publish_tsdf_pointclouds_(false),
      publish_esdf_pointclouds_(false),
      publish_tsdf_map_(false),
      publish_esdf_map_(false) {
  // Set up map and integrator.
  tsdf_map_.reset(new TsdfMap(tsdf_config));
  esdf_map_.reset(new EsdfMap(esdf_config));

  mesh_layer_.reset(new MeshLayer(tsdf_map_->block_size()));
  mesh_integrator_.reset(new MeshIntegrator<TsdfVoxel>(
      mesh_config, tsdf_map_->getTsdfLayerPtr(), mesh_layer_.get()));

  setupRos();
}

void CentralServer::setupRos() {
  nh_private_.param("world_frame", world_frame_, world_frame_);
  nh_private_.param("publish_tsdf_pointclouds", publish_tsdf_pointclouds_, publish_tsdf_pointclouds_);
  nh_private_.param("publish_esdf_pointclouds", publish_esdf_pointclouds_, publish_esdf_pointclouds_);
  nh_private_.param("publish_tsdf_map", publish_tsdf_map_, publish_tsdf_map_);
  nh_private_.param("publish_esdf_map", publish_esdf_map_, publish_esdf_map_);
  nh_private_.param("verbose", verbose_, verbose_);

  // Mesh settings.
  std::string color_mode("");
  nh_private_.param("color_mode", color_mode, color_mode);
  color_mode_ = getColorModeFromString(color_mode);

  tsdf_map_sub1_ = nh_private_.subscribe("/uav1/voxblox_node/tsdf_map_out", 1, &CentralServer::tsdfMapCallback, this);
  tsdf_map_sub2_ = nh_private_.subscribe("/uav2/voxblox_node/tsdf_map_out", 1, &CentralServer::tsdfMapCallback, this);
  tsdf_map_sub3_ = nh_private_.subscribe("/uav3/voxblox_node/tsdf_map_out", 1, &CentralServer::tsdfMapCallback, this);

  esdf_map_sub1_ = nh_private_.subscribe("/uav1/voxblox_node/esdf_map_out", 1, &CentralServer::esdfMapCallback, this);
  esdf_map_sub2_ = nh_private_.subscribe("/uav2/voxblox_node/esdf_map_out", 1, &CentralServer::esdfMapCallback, this);
  esdf_map_sub3_ = nh_private_.subscribe("/uav3/voxblox_node/esdf_map_out", 1, &CentralServer::esdfMapCallback, this);

  tsdf_merged_pointcloud_pub_ = nh_private_.advertise<pcl::PointCloud<pcl::PointXYZI> >("tsdf_pointcloud", 1, true);
  esdf_merged_pointcloud_pub_ = nh_private_.advertise<pcl::PointCloud<pcl::PointXYZI> >("esdf_pointcloud", 1, true);

  tsdf_merged_map_pub_ = nh_private_.advertise<voxblox_msgs::Layer>("merged_tsdf_map", 1, true);
  esdf_merged_map_pub_ = nh_private_.advertise<voxblox_msgs::Layer>("merged_esdf_map", 1, true);

  merged_mesh_pub_ = nh_private_.advertise<voxblox_msgs::Mesh>("merged_mesh", 1, true);

  save_map_srv_ = nh_private_.advertiseService(
      "save_map", &CentralServer::saveMapCallback, this);

  double update_mesh_every_n_sec = 1.0;
  nh_private_.param("update_mesh_every_n_sec", update_mesh_every_n_sec,
                    update_mesh_every_n_sec);

  if (update_mesh_every_n_sec > 0.0) {
    update_mesh_timer_ =
        nh_private_.createTimer(ros::Duration(update_mesh_every_n_sec),
                                &CentralServer::updateMergedMeshEvent, this);
  }

  double publish_tsdf_map_every_n_sec = 1.0;
  nh_private_.param("publish_tsdf_map_every_n_sec", publish_tsdf_map_every_n_sec,
                      publish_tsdf_map_every_n_sec);

  if (publish_tsdf_map_every_n_sec > 0.0) {
      publish_tsdf_map_timer_ =
          nh_private_.createTimer(ros::Duration(publish_tsdf_map_every_n_sec),
                                  &CentralServer::publishTsdfMergedMapEvent, this);
  }

  double publish_esdf_map_every_n_sec = 1.0;
  nh_private_.param("publish_esdf_map_every_n_sec", publish_esdf_map_every_n_sec,
                      publish_esdf_map_every_n_sec);

  if (publish_esdf_map_every_n_sec > 0.0) {
      publish_esdf_map_timer_ =
          nh_private_.createTimer(ros::Duration(publish_esdf_map_every_n_sec),
                                  &CentralServer::publishEsdfMergedMapEvent, this);
  }
}

void CentralServer::tsdfMapCallback(const voxblox_msgs::Layer& layer_msg) {
  if (!publish_tsdf_map_) {
      return;
  }
  //Layer<TsdfVoxel>* layer_msg_converted = nullptr;
  Layer<TsdfVoxel> layer_msg_converted(tsdf_map_->getTsdfLayerPtr()->voxel_size(), tsdf_map_->getTsdfLayerPtr()->voxels_per_side());
  bool success = deserializeMsgToLayer<TsdfVoxel>(layer_msg, &layer_msg_converted);
  if (!success) {
    ROS_ERROR_THROTTLE(10, "Got an invalid TSDF map message!");
  } else {
    voxblox::mergeLayerAintoLayerB(layer_msg_converted, tsdf_map_->getTsdfLayerPtr());
  }
  //publishTsdfMergedMap();
}

void CentralServer::esdfMapCallback(const voxblox_msgs::Layer& layer_msg) {
  if (!publish_esdf_map_) {
      return;
  }
  //Layer<EsdfVoxel>* layer_msg_converted = nullptr;
  Layer<EsdfVoxel> layer_msg_converted(esdf_map_->getEsdfLayerPtr()->voxel_size(), esdf_map_->getEsdfLayerPtr()->voxels_per_side());
  bool success = deserializeMsgToLayer<EsdfVoxel>(layer_msg, &layer_msg_converted);
  if (!success) {
    ROS_ERROR_THROTTLE(10, "Got an invalid ESDF map message!");
  } else {
    voxblox::mergeLayers(layer_msg_converted, esdf_map_->getEsdfLayerPtr());
  }
  //publishEsdfMergedMap();
}

void CentralServer::publishTsdfPointclouds() {
  // Create a pointcloud with distance = intensity.
  pcl::PointCloud<pcl::PointXYZI> pointcloud;

  createDistancePointcloudFromTsdfLayer(tsdf_map_->getTsdfLayer(), &pointcloud);

  pointcloud.header.frame_id = world_frame_;
  tsdf_merged_pointcloud_pub_.publish(pointcloud);
}

void CentralServer::publishEsdfPointclouds() {
  // Create a pointcloud with distance = intensity.
  pcl::PointCloud<pcl::PointXYZI> pointcloud;

  createDistancePointcloudFromEsdfLayer(esdf_map_->getEsdfLayer(), &pointcloud);

  pointcloud.header.frame_id = world_frame_;
  esdf_merged_pointcloud_pub_.publish(pointcloud);
}

void CentralServer::updateMesh() {
  if (verbose_) {
    ROS_INFO("Updating mesh.");
  }

  timing::Timer generate_mesh_timer("mesh/update");
  constexpr bool only_mesh_updated_blocks = true;
  constexpr bool clear_updated_flag = true;
  mesh_integrator_->generateMesh(only_mesh_updated_blocks, clear_updated_flag);
  generate_mesh_timer.Stop();

  timing::Timer publish_mesh_timer("mesh/publish");

  voxblox_msgs::Mesh mesh_msg;
  generateVoxbloxMeshMsg(mesh_layer_, color_mode_, &mesh_msg);
  mesh_msg.header.frame_id = world_frame_;
  merged_mesh_pub_.publish(mesh_msg);

  publish_mesh_timer.Stop();
}

bool CentralServer::saveMap(const std::string& file_path) {
  constexpr bool kClearFile = false;
  return io::SaveLayer(tsdf_map_->getTsdfLayer(), file_path) && io::SaveLayer(esdf_map_->getEsdfLayer(), file_path, kClearFile);  
}

void CentralServer::publishTsdfMergedMap() {
  timing::Timer publish_tsdf_map_timer("combined_map/publish_tsdf");
  voxblox_msgs::Layer merged_layer_msg;
  serializeLayerAsMsg<voxblox::TsdfVoxel>(*(tsdf_map_->getTsdfLayerPtr()), false, &merged_layer_msg);
  tsdf_merged_map_pub_.publish(merged_layer_msg);
  if (publish_tsdf_pointclouds_) {
    publishTsdfPointclouds();
  }
  if (verbose_) {
      ROS_INFO("Finished TSDF merging, have %lu blocks.",
            tsdf_map_->getTsdfLayer().getNumberOfAllocatedBlocks());
  }
  publish_tsdf_map_timer.Stop();
}

void CentralServer::publishEsdfMergedMap() {
  timing::Timer publish_esdf_map_timer("combined_map/publish_esdf");
  voxblox_msgs::Layer merged_layer_msg;
  serializeLayerAsMsg<voxblox::EsdfVoxel>(*(esdf_map_->getEsdfLayerPtr()), false, &merged_layer_msg);
  esdf_merged_map_pub_.publish(merged_layer_msg);
  if (publish_esdf_pointclouds_) {
    publishEsdfPointclouds();
  }
  if (verbose_) {
      ROS_INFO("Finished ESDF merging, have %lu blocks.",
            esdf_map_->getEsdfLayer().getNumberOfAllocatedBlocks());
  }
  publish_esdf_map_timer.Stop();
}

void CentralServer::publishTsdfMergedMapEvent(const ros::TimerEvent& /*event*/) {
  publishTsdfMergedMap();
}

void CentralServer::publishEsdfMergedMapEvent(const ros::TimerEvent& /*event*/) {
  publishEsdfMergedMap();
}

void CentralServer::updateMergedMeshEvent(const ros::TimerEvent& /*event*/) {
  updateMesh();
}

bool CentralServer::saveMapCallback(voxblox_msgs::FilePath::Request& request,
                                    voxblox_msgs::FilePath::Response&
                                    /*response*/) {  // NOLINT
  return saveMap(request.file_path);
}

} // namespace voxblox