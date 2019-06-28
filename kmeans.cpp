#include <pcl/io/ply_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/ml/kmeans.h>

int main(int argc, char** argv)
{
    
  pcl::PCDReader reader;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
  reader.read ("results/1_res/cloud_cluster_0.pcd", *cloud);
  std::cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl; //*

  pcl::Kmeans real(static_cast<int> (cloud->points.size()), 3);
  real.setClusterSize(2); //it is important that you set this term appropriately for your application
  for (size_t i = 0; i < cloud->points.size(); i++)
  {
      std::vector<float> data(3);
      data[0] = cloud->points[i].x;
      data[1] = cloud->points[i].y;
      data[2] = cloud->points[i].z;
      real.addDataPoint(data);
  }
  
  real.kMeans();
  // get the cluster centroids 
  pcl::Kmeans::Centroids centroids = real.get_centroids();
  std::cout << "points in total Cloud : " << cloud->points.size() << std::endl;
  std::cout << "centroid count: " << centroids.size() << std::endl;
  for (int i = 0; i<centroids.size(); i++)
  {
      std::cout << i << "_cent output: x: " << centroids[i][0] << " ,";
      std::cout << "y: " << centroids[i][1] << " ,";
      std::cout << "z: " << centroids[i][2] << std::endl;
  }

}
