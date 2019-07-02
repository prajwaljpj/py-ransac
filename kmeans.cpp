#include <pcl/common/common_headers.h>

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
#include <pcl/visualization/pcl_visualizer.h>


pcl::visualization::PCLVisualizer::Ptr simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, pcl::PointCloud<pcl::PointXYZ>::ConstPtr centroi)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(centroi, 0, 255, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "Segmented_planes");
  viewer->addPointCloud<pcl::PointXYZ> (centroi, single_color, "Centroids");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Segmented_planes");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "Centroids");
  /* viewer->addCoordinateSystem (1.0); */
  viewer->initCameraParameters ();
  return (viewer);
}

void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                            void* viewer_void)
{
  pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);
  if (event.getKeySym () == "c" && event.keyDown ())
  {
    std::cout << "c was pressed => Removing coordinate system" << std::endl;
    viewer->removeCoordinateSystem ();
  }
}

int main(int argc, char** argv)
{
    
  pcl::PCDReader reader;
  pcl::PCDWriter writer;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
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
  pcl::PointCloud<pcl::PointXYZ>::Ptr centroidpc (new pcl::PointCloud<pcl::PointXYZ>);
  centroidpc->width = centroids.size();
  centroidpc->height = 1;
  centroidpc->points.resize (centroidpc->width * centroidpc->height);
  
  for (int i = 0; i<centroids.size(); i++)
  {
      /* pcl::PointCloud<pcl::PointXYZ>::Ptr kmeans_cluster (new pcl::PointCloud<pcl::PointXYZ>); */
      /* kmeans_cluster->points.push_back (real.PointsToClusters); */
      /* std::cout<<real.PointsToClusters()<<std::endl; */
      /* std::vector<pcl::Kmeans::ClusterId> centroid_points = real.PointsToClusters(); */

      /* real.PointsToClusters(asdf); */

//      for (int j = 0; j<asdf.size(); j++)
//      {
//          /* kmeans_cluster = real.PointsToClusters; */
//          std::cout << asdf[j]<< std::endl;
//      }
      std::stringstream ss;
      ss << "results/1_res/kmeans_cluster_" << i << ".pcd";
      /* writer.write<pcl::PointXYZ> (ss.str(), *kmeans_cluster, false); */
      std::cout << i << "_cent output: x: " << centroids[i][0] << " ,";
      std::cout << "y: " << centroids[i][1] << " ,";
      std::cout << "z: " << centroids[i][2] << std::endl;
      centroidpc->points[i].x = centroids[i][0];
      centroidpc->points[i].y = centroids[i][1];
      centroidpc->points[i].z = centroids[i][2];
  }
  pcl::visualization::PCLVisualizer::Ptr viewer;
  cout<< "came here"<<std::endl;
  viewer = simpleVis (cloud, centroidpc);
  viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)viewer.get ());
  while (!viewer->wasStopped ())
  {
      viewer->spinOnce (100);
      boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }

  return(0);
}
