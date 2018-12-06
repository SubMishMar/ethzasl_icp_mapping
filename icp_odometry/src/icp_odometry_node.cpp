#include "ros/ros.h"
#include "ros/console.h"
#include "pointmatcher/PointMatcher.h"
#include "pointmatcher/Timer.h"

#include "nabo/nabo.h"

#include "pointmatcher_ros/point_cloud.h"
#include "pointmatcher_ros/transform.h"
#include "pointmatcher_ros/get_params_from_server.h"
#include "pointmatcher_ros/ros_logger.h"

#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h"
#include "tf_conversions/tf_eigen.h"
#include "tf/transform_listener.h"
#include "eigen_conversions/eigen_msg.h"

using namespace std;
using namespace PointMatcherSupport;

typedef PointMatcher<float> PM;

class ICPOdometry {
	typedef PM::DataPoints DP;
	typedef PM::Matches Matches;
	
	typedef typename Nabo::NearestNeighbourSearch<float> NNS;
	typedef typename NNS::SearchType NNSearchType;

	ros::NodeHandle n;

	// Subscriber
	ros::Subscriber cloudSub;

	// libpointmatcher
	PM::DataPointsFilters inputFilters;
	PM::DataPointsFilters mapPreFilters;
	PM::DataPointsFilters mapPostFilters;
	PM::DataPoints *mapPointCloud;
	PM::ICPSequence icp;
	shared_ptr<PM::Transformation> transformation;
	shared_ptr<PM::DataPointsFilter> radiusFilter;	 

	PM::TransformationParameters T_odom_to_map;
	PM::TransformationParameters T_localMap_to_map;
	PM::TransformationParameters T_odom_to_scanner;

public:
	ICPOdometry(ros::NodeHandle n);
protected:
	void pcdCallback(const sensor_msgs::PointCloud2& cloudMsgIn);
	void processCloud(unique_ptr<DP> cloud, const std::string& scannerFrame, const ros::Time& stamp, uint32_t seq);
};

ICPOdometry::ICPOdometry(ros::NodeHandle n) {
	cloudSub = n.subscribe("/vertical_laser_3d", 10, &ICPOdometry::pcdCallback, this);
}

void ICPOdometry::pcdCallback(const sensor_msgs::PointCloud2& cloudMsgIn) {
	unique_ptr<DP> cloud(new DP(PointMatcher_ros::rosMsgToPointMatcherCloud<float>(cloudMsgIn)));
	processCloud(move(cloud), cloudMsgIn.header.frame_id, cloudMsgIn.header.stamp, cloudMsgIn.header.seq);
}

void ICPOdometry::processCloud(unique_ptr<DP> newPointCloud, const std::string& scannerFrame, const ros::Time& stamp, uint32_t seq){
	// Convert point cloud
	const size_t goodCount(newPointCloud->features.cols());
	if (goodCount == 0)	{
		ROS_ERROR("[ICP] Found no good points in the cloud");
		return;
	} 
	const int dimp1(newPointCloud->features.rows());	
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "ICPOdometry");
	ros::NodeHandle n;
	ICPOdometry icpodometry(n);
	ros::spin();
	return 0;	
}