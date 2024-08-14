#ifndef VOXBLOX_ROS_CENTRAL_SERVER_H_
#define VOXBLOX_ROS_CENTRAL_SERVER_H_

#include <memory>
#include <queue>
#include <string>

#include <pcl/conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>

#include <voxblox/alignment/icp.h>
#include <voxblox/core/tsdf_map.h>
#include <voxblox/core/esdf_map.h>
#include <voxblox/integrator/merge_integration.h>
#include <voxblox/integrator/tsdf_integrator.h>
#include <voxblox/io/layer_io.h>
#include <voxblox/io/mesh_ply.h>
#include <voxblox/mesh/mesh_integrator.h>
#include <voxblox/utils/color_maps.h>
#include <voxblox_msgs/FilePath.h>
#include <voxblox_msgs/Mesh.h>

#include "voxblox_ros/mesh_vis.h"
#include "voxblox_ros/ptcloud_vis.h"
#include "voxblox_ros/transformer.h"

#include "voxblox_ros/tsdf_server.h"
#include "voxblox_ros/esdf_server.h"

namespace voxblox {

class CentralServer {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  CentralServer(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
  CentralServer(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
                const EsdfMap::Config& esdf_config,
                const TsdfMap::Config& tsdf_config,
                const MeshIntegratorConfig& mesh_config);
  virtual ~CentralServer() {}

  void setupRos();

  void tsdfMapCallback(const voxblox_msgs::Layer& layer_msg);
  void esdfMapCallback(const voxblox_msgs::Layer& layer_msg);

  void publishTsdfPointclouds();
  void publishEsdfPointclouds();

  virtual void updateMesh();
  virtual bool saveMap(const std::string& file_path);

  void publishTsdfMergedMap();
  void publishEsdfMergedMap();

  void publishTsdfMergedMapEvent(const ros::TimerEvent& event);
  void publishEsdfMergedMapEvent(const ros::TimerEvent& event);
  void updateMergedMeshEvent(const ros::TimerEvent& event);
  
  bool saveMapCallback(voxblox_msgs::FilePath::Request& request,     // NOLINT
                       voxblox_msgs::FilePath::Response& response);  // NOLINT

 protected:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  bool verbose_;
  std::string world_frame_;

  bool publish_tsdf_pointclouds_;
  bool publish_esdf_pointclouds_;
  bool publish_tsdf_map_;
  bool publish_esdf_map_;

  ColorMode color_mode_;
  
  ros::Subscriber tsdf_map_sub1_, tsdf_map_sub2_, tsdf_map_sub3_;
  ros::Subscriber esdf_map_sub1_, esdf_map_sub2_, esdf_map_sub3_;

  ros::Publisher tsdf_merged_pointcloud_pub_;
  ros::Publisher esdf_merged_pointcloud_pub_;

  ros::Publisher tsdf_merged_map_pub_;
  ros::Publisher esdf_merged_map_pub_;
  ros::Publisher merged_mesh_pub_;

  ros::ServiceServer save_map_srv_;

  ros::Timer update_mesh_timer_;
  ros::Timer publish_tsdf_map_timer_;
  ros::Timer publish_esdf_map_timer_;

  voxblox::TsdfMap::Ptr tsdf_map_;
  voxblox::EsdfMap::Ptr esdf_map_;

  // Mesh accessories.
  std::shared_ptr<MeshLayer> mesh_layer_;
  std::unique_ptr<MeshIntegrator<TsdfVoxel>> mesh_integrator_;
};

}  // namespace voxblox

#endif  // VOXBLOX_ROS_CENTRAL_SERVER_H_