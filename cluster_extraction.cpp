#include <pcl/common/common_headers.h>

#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/pcl_visualizer.h>


pcl::visualization::PCLVisualizer::Ptr simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloudvis){
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  /* pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(centroi, 0, 255, 0); */
  viewer->addPointCloud<pcl::PointXYZ> (cloudvis, "Segmented_planes");
  /* viewer->addPointCloud<pcl::PointXYZ> (centroi, single_color, "Centroids"); */
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Segmented_planes");
  /* viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "Centroids"); */
  /* viewer->addCoordinateSystem (1.0); */
  viewer->initCameraParameters ();
  return (viewer);
}

double point2planedistnace(pcl::PointXYZ pt, pcl::ModelCoefficients::Ptr coefficients){
    double f1 = fabs(coefficients->values[0]*pt.x+coefficients->values[1]*pt.y+coefficients->values[2]*pt.z+coefficients->values[3]);
    double f2 = sqrt(pow(coefficients->values[0],2)+pow(coefficients->values[1],2)+pow(coefficients->values[2],2));
    return f1/f2;
}


class ColorMap{
public:
    ColorMap(double mn, double mx): mn(mn), mx(mx){}
    void setMinMax(double min, double max){ mn = min; mx = max;}
    void setMin(double min){mn = min;}
    void setMax(double max){mx = max;}
    void getColor(double c,uint8_t& R, uint8_t& G, uint8_t& B){
        double normalized = (c - mn)/(mx-mn) * 2 - 1;
        R = (int) (base(normalized - 0.5) * 255);
        G = (int) (base(normalized) * 255);
        B = (int) (base(normalized + 0.5) * 255);
    }
    void getColor(double c, double &rd, double &gd, double &bd){
        uint8_t r;
        uint8_t g;
        uint8_t b;
        getColor(c,r,g,b);
        rd = (double)r/255;
        gd = (double)g/255;
        bd = (double)b/255;
    }
    uint32_t getColor(double c){
        uint8_t r;
        uint8_t g;
        uint8_t b;
        getColor(c,r,g,b);
        return ((uint32_t)r<<16|(uint32_t)g<<8|(uint32_t)b);
    }


private:
    double interpolate(double val, double y0, double x0, double y1, double x1){
        return (val - x0)*(y1-y0)/(x1-x0) + y0;
    }
    double base(double val){
        if (val <= -0.75) return 0;
        else if (val <= -0.25) return interpolate(val,0,-0.75,1,-0.25);
        else if (val <= 0.25) return 1;
        else if (val <= 0.75) return interpolate(val,1.0,0.25,0.0,0.75);
        else return 0;
    }
private:
    double mn,mx;
};


class Color{
private:
    uint8_t r;
    uint8_t g;
    uint8_t b;

public:
    Color(uint8_t R,uint8_t G,uint8_t B):r(R),g(G),b(B){

    }

    void getColor(uint8_t &R,uint8_t &G,uint8_t &B){
        R = r;
        G = g;
        B = b;
    }
    void getColor(double &rd, double &gd, double &bd){
        rd = (double)r/255;
        gd = (double)g/255;
        bd = (double)b/255;
    }
    uint32_t getColor(){
        cout<<r<<'\n'<<g<<'\n'<<b<<'\n'<< endl;
        return ((uint32_t)r<<16|(uint32_t)g<<8|(uint32_t)b);
    }
};


std::vector<Color> createColors(){
  std::vector<Color> colors;
  uint8_t r = 0;
  uint8_t g = 0;
  uint8_t b = 0;
  for (int i=0;i<20;i++){
    while (r<70 && g < 70 && b < 70){
      r = rand()%(255);
      g = rand()%(255);
      b = rand()%(255);
    }
    Color c(r,g,b);
    r = 0;
    g = 0;
    b = 0;
    colors.push_back(c);
  }
}


void pointCloudCb(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    // Some work to be done 
    float _max_distance = 0.10;
    int _min_percentage = 5;
    bool _color_pc_with_error = false;
    std::string _name = "1.pcd";
    std::vector<Color> colors = createColors();

    // Convert to pcl point cloud
    /* pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_msg (new pcl::PointCloud<pcl::PointXYZ>); */
    /* pcl::fromROSMsg(*msg,*cloud_msg); */
    /* ROS_DEBUG("%s: new ponitcloud (%i,%i)(%zu)",_name.c_str(),cloud_msg->width,cloud_msg->height,cloud_msg->size()); */

    // Filter cloud
    /* pcl::PassThrough<pcl::PointXYZ> pass; */
    /* pass.setInputCloud(cloud_msg); */
    /* pass.setFilterFieldName ("z"); */
    /* pass.setFilterLimits(0.001,10000); */
    /* pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>); */
    /* pass.filter (*cloud); */

    // Get segmentation ready
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold(_max_distance);

    // Create pointcloud to publish inliers
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_pub(new pcl::PointCloud<pcl::PointXYZRGB>);
    int original_size(cloud->height*cloud->width);
    int n_planes(0);
    cout << "here?"<< endl;
    while (cloud->height*cloud->width>original_size*_min_percentage/100){

        // Fit a plane
        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);

        // Check result
        if (inliers->indices.size() == 0)
            break;

        // Iterate inliers
        double mean_error(0);
        double max_error(0);
        double min_error(100000);
        std::vector<double> err;
        for (int i=0;i<inliers->indices.size();i++){

            // Get Point
            pcl::PointXYZ pt = cloud->points[inliers->indices[i]];

            // Compute distance
            double d = point2planedistnace(pt,coefficients)*1000;// mm
            err.push_back(d);

            // Update statistics
            mean_error += d;
            if (d>max_error) max_error = d;
            if (d<min_error) min_error = d;

        }
        mean_error/=inliers->indices.size();

        // Compute Standard deviation
        ColorMap cm(min_error,max_error);
        double sigma(0);
        cout<< "in while?" << endl;
        for (int i=0;i<inliers->indices.size();i++){

            sigma += pow(err[i] - mean_error,2);

            // Get Point
            cout<< "sigma later?"<< endl;
            pcl::PointXYZ pt = cloud->points[inliers->indices[i]];

            // Copy point to noew cloud
            pcl::PointXYZRGB pt_color;
            cout<<"got inliers?"<< endl;
            pt_color.x = pt.x;
            pt_color.y = pt.y;
            pt_color.z = pt.z;
            cout<<"got points color?"<<endl;
            uint32_t rgb;
            if (_color_pc_with_error){
                cout<<"in if condition?"<<endl;
                rgb = cm.getColor(err[i]);}
            else{
                cout<<"in else condition?"<<endl;
                rgb = colors[n_planes].getColor();}
                cout<<"not in ifelse"<<endl;
            pt_color.rgb = *reinterpret_cast<float*>(&rgb);
            cloud_pub->points.push_back(pt_color);
            cout<<"in for?"<<endl;

        }
        sigma = sqrt(sigma/inliers->indices.size());

        // Extract inliers
        extract.setInputCloud(cloud);
        extract.setIndices(inliers);
        extract.setNegative(true);
        pcl::PointCloud<pcl::PointXYZ> cloudF;
        extract.filter(cloudF);
        cloud->swap(cloudF);

        // Display infor
        /* ROS_INFO("%s: fitted plane %i: %fx%s%fy%s%fz%s%f=0 (inliers: %zu/%i)", */
        /*          _name.c_str(),n_planes, */
        /*          coefficients->values[0],(coefficients->values[1]>=0?"+":""), */
        /*          coefficients->values[1],(coefficients->values[2]>=0?"+":""), */
        /*          coefficients->values[2],(coefficients->values[3]>=0?"+":""), */
        /*          coefficients->values[3], */
        /*          inliers->indices.size(),original_size); */
        /* ROS_INFO("%s: mean error: %f(mm), standard deviation: %f (mm), max error: %f(mm)",_name.c_str(),mean_error,sigma,max_error); */
        /* ROS_INFO("%s: poitns left in cloud %i",_name.c_str(),cloud->width*cloud->height); */


        cout<< _name<<": fitted plane "<< n_planes << ": " << coefficients->values[0] << "x" << (coefficients->values[1]>=0?"+":"") << coefficients->values[1] <<"y" << (coefficients->values[2]>=0?"+":"") << coefficients->values[2] << "z"<< (coefficients->values[3]>=0?"+":"") << coefficients->values[3] << "=0 (inliers: " << inliers->indices.size() << "u/" << original_size << ")" << std::endl;
        cout << _name << ": mean error: " << mean_error << "(mm), standard deviation: " << sigma << " (mm), max error: " << max_error << "(mm)" << endl;
        cout << _name << ": points left in cloud " << cloud->width*cloud->height << endl;

        // Nest iteration
        n_planes++;
    }

    // Publish points
    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZRGB> ("results/1_res/points.pcd", *cloud_pub, false);
}



int 
main (int argc, char** argv)
{
  
   
  /* // selecting GPU and prining info */
  /* /1* int device = 0; *1/ */
  /* /1* pcl::gpu::setDevice (device); *1/ */
  /* /1* pcl::gpu::printShortCudaDeviceInfo (device); *1/ */

  /* // Read in the cloud data */

  /* /1* pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>); *1/ */
  /* /1* pcl::PLYReader reader; *1/ */
  /* /1* reader.read("data/1.ply", *cloud); *1/ */
  /* /1* pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>); *1/ */
  /* /1* pcl::PCDReader reader; *1/ */
  /* /1* pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>); *1/ */
  /* /1* reader.read ("data/table_scene_lms400.pcd", *cloud); *1/ */
  pcl::PCDReader reader;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudin (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
  reader.read ("data/1.pcd", *cloudin);
  std::cout << "PointCloud before filtering has: " << cloudin->points.size () << " data points." << std::endl; 
  pointCloudCb(cloudin);



  /* // Create the filtering object: downsample the dataset using a leaf size of 1cm */
  /* pcl::VoxelGrid<pcl::PointXYZ> vg; */
  /* pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>); */
  /* vg.setInputCloud (cloud); */
  /* // for 1_pc */
  /* vg.setLeafSize (0.0075f, 0.0075f, 0.0075f); */
  /* // for 2_pc */
  /* /1* vg.setLeafSize (0.001f, 0.001f, 0.001f); *1/ */
  /* vg.filter (*cloud_filtered); */
  /* std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; */

  /* // Create the segmentation object for the planar model and set all the parameters */
  /* pcl::SACSegmentation<pcl::PointXYZ> seg; */
  /* pcl::PointIndices::Ptr inliers (new pcl::PointIndices); */
  /* pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients); */
  /* pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ()); */
  /* pcl::PCDWriter writer; */
  /* seg.setOptimizeCoefficients (true); */
  /* seg.setModelType (pcl::SACMODEL_PLANE); */
  /* seg.setMethodType (pcl::SAC_RANSAC); */
  /* seg.setMaxIterations (1000); */
  /* seg.setDistanceThreshold (0.10); */

  /* // Write the filtered pointcloud */
  /* /1* writer.write<pcl::PointXYZ> ("results/iisc/cloud_filtered.pcd", *cloud_filtered, false); //1* *1/ */

  /* int i=0, nr_points = (int) cloud_filtered->points.size (); */
  /* while (cloud_filtered->points.size () > 0.3 * nr_points) */
  /* { */
  /*   // Segment the largest planar component from the remaining cloud */
  /*   seg.setInputCloud (cloud_filtered); */
  /*   seg.segment (*inliers, *coefficients); */
  /*   if (inliers->indices.size () == 0) */
  /*   { */
  /*     std::cout << "Could not estimate a planar model for the given dataset." << std::endl; */
  /*     break; */
  /*   } */

  /*   // Extract the planar inliers from the input cloud */
  /*   pcl::ExtractIndices<pcl::PointXYZ> extract; */
  /*   extract.setInputCloud (cloud_filtered); */
  /*   extract.setIndices (inliers); */
  /*   extract.setNegative (false); */

  /*   // Get the points associated with the planar surface */
  /*   extract.filter (*cloud_plane); */
  /*   std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl; */

  /*   // Remove the planar inliers, extract the rest */
  /*   extract.setNegative (true); */
  /*   extract.filter (*cloud_f); */
  /*   *cloud_filtered = *cloud_f; */
  /* } */

  /* // Creating the KdTree object for the search method of the extraction */
  /* pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>); */
  /* tree->setInputCloud (cloud_filtered); */

  /* std::vector<pcl::PointIndices> cluster_indices; */
  /* pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec; */
  /* ec.setClusterTolerance (0.01); // 1cm */
  /* ec.setMinClusterSize (100); */
  /* ec.setMaxClusterSize (10000); */
  /* ec.setSearchMethod (tree); */
  /* ec.setInputCloud (cloud_filtered); */
  /* ec.extract (cluster_indices); */

  /* int j = 0; */
  /* for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it) */
  /* { */
  /*   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>); */
  /*   for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit) */
  /*     cloud_cluster->points.push_back (cloud_filtered->points[*pit]); */
  /*   cloud_cluster->width = cloud_cluster->points.size (); */
  /*   cloud_cluster->height = 1; */
  /*   cloud_cluster->is_dense = true; */

  /*   std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl; */
  /*   std::stringstream ss; */
  /*   ss << "results/iisc/cloud_cluster_" << j << ".pcd"; */
  /*   writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); */
  /*   j++; */
  /* } */

  return (0);
}
