#include <iostream>
#include <thread>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/console/parse.h>
#include <pcl/common/common.h>
#include <pcl/common/distances.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <glog/logging.h>
#include <gflags/gflags.h>

using namespace std::chrono_literals;
typedef pcl::PointXYZI Point;
typedef pcl::PointCloud<Point> PointCloud;
static std::shared_ptr<pcl::visualization::PCLVisualizer> _pcl_viewer;
pcl::PointCloud<pcl::PointXYZI>::Ptr clicked_points_3d(new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
int num = 0;

static bool g_viewer_stopped = false;    // flag viewer stopped by user
DEFINE_string(scan_paths, "", "pcd files, separated by space, in same order as image paths (required)");
template<typename Out>

void split(const std::string &s, char delim, Out result) {
    std::stringstream ss;
    ss.str(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        if (!item.empty())
            *(result++) = item;
    }
}

void viewKeyboardCallback(const pcl::visualization::KeyboardEvent &event, void* view) {
    std::shared_ptr<pcl::visualization::PCLVisualizer> *viewer = static_cast<std::shared_ptr<pcl::visualization::PCLVisualizer> *>(view);

    if(event.getKeySym() == "q"){
        g_viewer_stopped = true;
    }
}

void lidarViewer(std::vector<std::string>& scan_paths) {
     pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
     viewer->setBackgroundColor (0, 0, 0);
     viewer->addCoordinateSystem (1.0, "axes");
     
     viewer->registerKeyboardCallback(viewKeyboardCallback, (void*)&viewer);

    for (size_t velo_frame=0; velo_frame<scan_paths.size(); velo_frame++) {
        LOG(INFO) << "velo frame " << velo_frame;
        PointCloud::Ptr cloud(new PointCloud);
        pcl::PointCloud<pcl::PointXYZI> tmp;
        CHECK(-1 != pcl::io::loadPCDFile<pcl::PointXYZI>(scan_paths[velo_frame], tmp)) << "could not load " << scan_paths[velo_frame];
        
        pcl::copyPointCloud(tmp, *cloud);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> single_color(cloud, 0, 255, 0);
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        viewer->addPointCloud<pcl::PointXYZI> (cloud, single_color, "sample cloud");

        viewer->spinOnce(500);
        //boost::this_thread::sleep(boost::posix_time::milliseconds(50));
        std::this_thread::sleep_for(10ms);

        if (g_viewer_stopped) {
            LOG(INFO) << "user input skipping rest frames";
            break;
            }
    }
}

pcl::visualization::PCLVisualizer::Ptr customColourVis (pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> single_color(cloud, 0, 255, 0);
  viewer->addPointCloud<pcl::PointXYZI> (cloud, single_color, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem (1.0);

  return (viewer);
}

void pp_callback(const pcl::visualization::AreaPickingEvent& event, void* args)
{

    // pcl::PointIndices::Ptr inliers;
    // inliers = pcl::PointIndices::Ptr(new pcl::PointIndices);
    // if (event.getPointsIndices(inliers->indices) != false) {
    // pcl::ExtractIndices<pcl::PointXYZI> extract;
    // extract.setIndices(inliers);
    // extract.setInputCloud(point_cloud_);
    // extract.setNegative(false); //矩形框选中的点

    // pcl::PointCloud<pcl::PointXYZI>::Ptr new_cloud_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
    // extract.filter(*new_cloud_);

    // pcl::copyPointCloud(*new_cloud_, *point_cloud_);
    // std::cout << "selected_cloud has " << point_cloud_->points.size() << " points." << std::endl;

    //  viewer_->removeAllPointClouds();
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> handle(point_cloud_, 50, 100, 139);
    // viewer_->addPointCloud(point_cloud_, handle);//显示矩形框选中的点云
    // }

	std::vector< int > indices;
	if (event.getPointsIndices(indices) == -1)
		return;
	for (int i = 0; i < indices.size(); ++i)
	{
		clicked_points_3d->points.push_back(cloud->points.at(indices[i]));
	}
	clicked_points_3d->height = 1;
	clicked_points_3d->width += indices.size();
    LOG(INFO) << "Selected point could amount is " << indices.size()/2;
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> red(clicked_points_3d, 255, 0, 0);
	//每添加一次点云都要取一次别名，不然只能选择一次
	std::stringstream ss;
	std::string cloudName;
	ss << num++;
	ss >> cloudName;
	cloudName += "_cloudName";
	//添加点云，并设置其显示半径
	viewer->addPointCloud(clicked_points_3d, red, cloudName);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, cloudName);
	// 保存点云
	pcl::PCDWriter writer;
	writer.write<pcl::PointXYZI>("plane.pcd", *clicked_points_3d);
}

int main (int argc, char** argv)
{
    google::ParseCommandLineFlags(&argc, &argv, true);
 
    std::vector<std::string> scan_paths;
    split(FLAGS_scan_paths, ' ', std::back_inserter(scan_paths));
    //CHECK(!scan_paths.empty()) << "at least one velo frame is needed";
    //lidarViewer(scan_paths);  

    CHECK(-1 != pcl::io::loadPCDFile<pcl::PointXYZI> ("/home/bryan/bags/validation/pandar_points_0001.pcd", *cloud)) << "Couldn't read file pandar_points.pcd";

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> single_color(cloud, 0, 255, 0);
    viewer->addPointCloud<pcl::PointXYZI> (cloud, single_color, "sample cloud");
    viewer->addCoordinateSystem (1.0);

	clicked_points_3d->width = 0;//初始化点云个数
    viewer->registerAreaPickingCallback(pp_callback, (void*)&cloud);

    while (!viewer->wasStopped ()) {
        viewer->spinOnce (100);
        std::this_thread::sleep_for(100ms);
    }



    //return 0;
}
