#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/pcl_visualizer.h>

/* #include "plane_fitter/algorithmParametersConfig.h" */

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
        return ((uint32_t)r<<16|(uint32_t)g<<8|(uint32_t)b);
    }
};

class planeFitter{
public:
    planeFitter(){
        initialize();
    }

    void initialize(){

        // Get node name
        _name = "1.pcd";

        // Get Publisher read
        /* _pub_inliers = _nh.advertise< sensor_msgs::PointCloud2 >("inliers",2); */

        // Subscriber
        /* _subs = _nh.subscribe("/sense3d/scan",1,&planeFitter::pointCloudCb,this); */

        // Get parameters
        /* ros::param::param<double>("~max_distance",_max_distance,0.005); */
        /* ros::param::param<double>("~min_percentage",_min_percentage,5); */
        /* ros::param::param<bool>("~color_pc_with_error",_color_pc_with_error,false); */
        _max_distance = 0.02;
        _min_percentage = 5;
        _color_pc_with_error = false;

        // Create dynamic reconfigure
        /* drCallback = boost::bind( &planeFitter::updateParameters, this, _1, _2); */
        /* dRserver.setCallback(drCallback); */

        // Create colors pallete
        createColors();

        // Inform initialized
        std::cout<<_name<<": node initialized."<< std::endl;
    }

    /* void updateParameters(plane_fitter::algorithmParametersConfig& config, uint32_t level){ */
    /*     _max_distance = config.max_distance/1000; */
    /*     _min_percentage = config.min_percentage_of_points; */
    /*     _color_pc_with_error = config.paint_with_error; */
    /* } */


    pcl::visualization::PCLVisualizer::Ptr simpleVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloudvis){
      // --------------------------------------------
      // -----Open 3D viewer and add point cloud-----
      // --------------------------------------------
      pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
      viewer->setBackgroundColor (0, 0, 0);
      /* pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(centroi, 0, 255, 0); */
      viewer->addPointCloud<pcl::PointXYZRGB> (cloudvis, "Segmented_planes");
      /* viewer->addPointCloud<pcl::PointXYZ> (centroi, single_color, "Centroids"); */
      viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Segmented_planes");
      /* viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "Centroids"); */
      /* viewer->addCoordinateSystem (1.0); */
      viewer->initCameraParameters ();
      return (viewer);
    }

    void pointCloudCb(){

        // Convert to pcl point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ori (new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPCDFile<pcl::PointXYZ> ("data/passthrough/1_pty.pcd", *cloud_ori) == -1) //* load the file
        {
            PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        }
        /* reader.read("data/1.pcd", *cloud_msg); */
        std::cout<<_name<<": new ponitcloud ("<<cloud_ori->width<<","<<cloud_ori->height<<" ("<<cloud_ori->size()<<"u)"<< std::endl;


        pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));

        // Filter cloud
        /* pcl::PassThrough<pcl::PointXYZ> pass; */
        /* pass.setInputCloud(cloud_msg); */
        /* pass.setFilterFieldName ("z"); */
        /* pass.setFilterLimits(0.001,10000); */
        /* pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>); */
        /* pass.filter (*cloud); */

        // Create the filtering object: downsample the dataset using a leaf size of 1cm
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        vg.setInputCloud (cloud_ori);
        // for 1_pc
        vg.setLeafSize (0.0075f, 0.0075f, 0.0075f);
        // for 2_pc
        /* vg.setLeafSize (0.001f, 0.001f, 0.001f); */
        vg.filter (*cloud);
        std::cout << "PointCloud after filtering has: " << cloud->points.size ()  << " data points." << std::endl;

        // Get segmentation ready
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        seg.setOptimizeCoefficients (true);
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setDistanceThreshold(_max_distance);
        seg.setMaxIterations(1000);

        std::cout << "**MODEL PARAMETERS**\n" << "Model Type: RANSAC Plane\n" << "Distance Threshold:" << seg.getDistanceThreshold() << '\n' << "Max Iteration:" << seg.getMaxIterations() << '\n' << "Probability:" << seg.getProbability() << std::endl;

        // Create pointcloud to publish inliers
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_pub(new pcl::PointCloud<pcl::PointXYZRGB>);
        int original_size = cloud->height*cloud->width;
        int n_planes(0);
        /* std::cout<<"reached 162"<<std::endl; */
        std::cout<<"original_size:"<<original_size<<'\n'<<"_min_percentage:"<<_min_percentage<<std::endl;
        while (cloud->height*cloud->width>original_size*_min_percentage/100){
            /* std::cout<<"entered While"<<std::endl; */

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
                /* std::cout<<"entered first for"<<std::endl; */

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
            for (int i=0;i<inliers->indices.size();i++){
                /* std::cout<<"entered second for"<<std::endl; */

                sigma += pow(err[i] - mean_error,2);

                // Get Point
                pcl::PointXYZ pt = cloud->points[inliers->indices[i]];

                // Copy point to noew cloud
                pcl::PointXYZRGB pt_color;
                pt_color.x = pt.x;
                pt_color.y = pt.y;
                pt_color.z = pt.z;
                uint32_t rgb;
                if (_color_pc_with_error)
                    rgb = cm.getColor(err[i]);
                else
                    rgb = colors[n_planes].getColor();
                pt_color.rgb = *reinterpret_cast<float*>(&rgb);
                cloud_pub->points.push_back(pt_color);

            }
            std::cout<<"exit for"<<std::endl;
            sigma = sqrt(sigma/inliers->indices.size());

            // Extract inliers
            extract.setInputCloud(cloud);
            extract.setIndices(inliers);
            extract.setNegative(true);
            pcl::PointCloud<pcl::PointXYZ> cloudF;
            extract.filter(cloudF);

            cloud->swap(cloudF);

            // Add planes
            std::stringstream plane_name;
            plane_name << "Plane_" << n_planes;
            viewer->addPlane(*coefficients, coefficients->values[0], coefficients->values[1], coefficients->values[2], plane_name.str());

            // Display infor
            std::cout<< _name<<": fitted plane "<< n_planes << ": " << coefficients->values[0] << "x" << (coefficients->values[1]>=0?"+":"") << coefficients->values[1] <<"y" << (coefficients->values[2]>=0?"+":"") << coefficients->values[2] << "z"<< (coefficients->values[3]>=0?"+":"") << coefficients->values[3] << "=0 (inliers: " << inliers->indices.size() << "u/" << original_size << ")" << std::endl;
            std::cout << _name << ": mean error: " << mean_error << "(mm), standard deviation: " << sigma << " (mm), max error: " << max_error << "(mm)" << std::endl;
            std::cout << _name << ": points left in cloud " << cloud->width*cloud->height << std::endl;
            std::cout << _name << ": number of points in cloud " << cloud->size() << std::endl;

            // Nest iteration
            n_planes++;
        }

        // Publish points
        /* sensor_msgs::PointCloud2 cloud_publish; */
        /* pcl::toROSMsg(*cloud_pub,cloud_publish); */
        /* cloud_publish.header = msg->header; */
        /* _pub_inliers.publish(cloud_publish); */

        /* pcl::io::savePCDFileASCII ("test_pcd.pcd", cloud_pub); */
        pcl::io::savePLYFileBinary("results/1_res/ransac/plane_points.ply", *cloud_pub);
        std::cerr << "Saved " << cloud_pub->points.size () << " data points to plane_points.pcd." << std::endl;
        /* pcl::PCDWriter writer; */
        /* writer.write<pcl::PointXYZ> ("results/1_res/points.pcd", *cloud_pub, false); */
        /* viewer = simpleVis(cloud_pub); */
        viewer->addPointCloud<pcl::PointXYZRGB> (cloud_pub, "Segmented_planes");
        /* viewer->addPointCloud<pcl::PointXYZ> (centroi, single_color, "Centroids"); */
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Segmented_planes");
        /* viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "Centroids"); */
        /* viewer->addCoordinateSystem (1.0); */
        viewer->initCameraParameters ();

        while (!viewer->wasStopped ())
        {
            viewer->spinOnce (100);
            boost::this_thread::sleep (boost::posix_time::microseconds (100000));
        }
        }

    void createColors(){
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

    /* void spin(){ */
    /*     ros::spin(); */
    /* } */

private:

    // Node
    /* ros::NodeHandle _nh; */
    std::string _name;

    // Publishers
    /* ros::Publisher _pub_inliers;// Display inliers for each plane */

    // Subscriber
    /* ros::Subscriber _subs; */

    // Algorithm parameters
    double _min_percentage;
    double _max_distance;
    bool _color_pc_with_error;

    // Colors
    std::vector<Color> colors;

    // Dynamic reconfigure
    /* dynamic_reconfigure::Server<plane_fitter::algorithmParametersConfig> dRserver; */
    /* dynamic_reconfigure::Server<plane_fitter::algorithmParametersConfig>::CallbackType drCallback; */
};

int main(int argc,char** argv){

    // Initialize ROS
    /* ros::init(argc,argv,"planeFitter"); */
    /* ros::NodeHandle nh("~"); */

    planeFitter pf;
    pf.pointCloudCb();
    /* pf.spin(); */

    return 0;
}
