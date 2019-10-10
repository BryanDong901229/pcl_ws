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

void veloCircleView(std::vector<std::string>& scan_paths) {
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
        std::this_thread::sleep_for(50ms);

        if (g_viewer_stopped) {
            LOG(INFO) << "user input skipping rest frames";
            break;
            }
    }
}

int main (int argc, char** argv)
{
    google::ParseCommandLineFlags(&argc, &argv, true);
 
    std::vector<std::string> scan_paths;
    split(FLAGS_scan_paths, ' ', std::back_inserter(scan_paths));
    CHECK(!scan_paths.empty()) << "at least one velo frame is needed";
    veloCircleView(scan_paths);  

    return 0;
}