/** File defining a cell converting vector<vector<vector<cv::Vec3f>>>
to vector<vector<PointCloud2>. Use all the points in all clusters for now.
*/

#include <ecto/ecto.hpp>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <opencv2/core/core.hpp>

//#include <Eigen/Core>
//#include <Eigen/Dense>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

using ecto::tendrils;

/** Ecto cells that creates messages containing what has been found by the tabletop pipeline
 */
struct ClusterConverter {
  static void declare_io(const tendrils& params, tendrils& inputs,
                         tendrils& outputs) {
    inputs.declare(&ClusterConverter::clusters3d_, "clusters3d",
                   "The clusters on top of the table.").required(true);
    inputs.declare(&ClusterConverter::image_message_,
                   "image_message", "the image message to get the header").required(true);

    outputs.declare<sensor_msgs::PointCloud2ConstPtr>(
      "cluster_pc", "The PointCloud2 message of the top cluster");
  }

  void configure(const tendrils& params, const tendrils& inputs,
                 const tendrils& outputs) {
  }

  int process(const tendrils& inputs, const tendrils& outputs) {

    sensor_msgs::PointCloud2 cloud_pc2;

    std::string frame_id;
    if (*image_message_)
      frame_id = (*image_message_)->header.frame_id;
    std_msgs::Header message_header;
    message_header.frame_id = frame_id;

    std::cout << "!!! Image Message Header Frame ID: " << frame_id << std::endl;

    convertCluster((*clusters3d_), message_header, cloud_pc2);

    outputs["cluster_pc"] << sensor_msgs::PointCloud2ConstPtr(new sensor_msgs::PointCloud2(cloud_pc2));

    return ecto::OK;
  }
private:

  void convertCluster(
    const std::vector<std::vector<std::vector<cv::Vec3f> > > &clusters,
    const std_msgs::Header& cloud_header,
    sensor_msgs::PointCloud2& cloud_pc2) {

    pcl::PointCloud<pcl::PointXYZ> cloud;

    // Use all clusters
    /*
    // The k-th table
    for (size_t k = 0; k < clusters.size(); k++) {
      // The i-th cluster
      for (size_t i = 0; i < clusters[k].size(); i++) {
        // The j-th point
        for (size_t j = 0; j < clusters[k][i].size(); j++) {
          pcl::PointXYZ pclPoint;
          pclPoint.x = clusters[k][i][j][0];
          pclPoint.y = clusters[k][i][j][1];
          pclPoint.z = clusters[k][i][j][2];
          cloud.push_back(pclPoint);
        }
      }
    }
    */

    // Use only the first cluster (k,i=0)
    size_t k, i = 0;
    for (size_t j = 0; j < clusters[k][i].size(); j++) {
      pcl::PointXYZ pclPoint;
      pclPoint.x = clusters[k][i][j][0];
      pclPoint.y = clusters[k][i][j][1];
      pclPoint.z = clusters[k][i][j][2];
      cloud.push_back(pclPoint);
    }

    pcl::toROSMsg(cloud, cloud_pc2);

    cloud_pc2.header = cloud_header;
  }

  /** The image message the initial data is from */
  ecto::spore<sensor_msgs::ImageConstPtr> image_message_;

  /** For each table, a vector of clusters */
  ecto::spore<std::vector<std::vector<std::vector<cv::Vec3f> > > > clusters3d_;
};

ECTO_CELL(tabletop_table, ClusterConverter,
          "ClusterConverter",
          "Converting vector<vector<vector<cv::Vec3f>>> to vector<vector<PointCloud2>");
