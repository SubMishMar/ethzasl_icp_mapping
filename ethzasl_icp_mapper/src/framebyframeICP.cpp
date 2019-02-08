#include <fstream>

#include <boost/version.hpp>
#include <boost/thread.hpp>
#if BOOST_VERSION >= 104100
    #include <boost/thread/future.hpp>
#endif // BOOST_VERSION >=  104100

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
#include "nav_msgs/Path.h"
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

    ros::NodeHandle& n;

    // Subscribers
    ros::Subscriber cloudSub;

    // Publisher
    ros::Publisher posePub;
    ros::Publisher pathPub;

    // libpointmatcher
    PM::DataPointsFilters inputFilters;
    PM::ICPSequence icp;
    shared_ptr<PM::Transformation> transformation;
    shared_ptr<PM::DataPointsFilters> radiusFilter;

    PM::TransformationParameters deltaT;

    // Parameters
    int minReadingPointCount;
    int inputQueueSize; 

    // Parameters for dynamic filtering
    const float priorDyn; //!< ratio. Prior to be dynamic when a new point is added >
    const float priorStatic; //!< ratio. Prior to be static when a new point is added >
    const float maxAngle; //!< in rad. Opening angle of a LASER beam >
    const float eps_a; //!< ratio. Error proportional to the laser distance >
    const float eps_d; //!< in meter. Fix error on the laser distance >
    const float alpha; //!< ratio. Propability of staying static given that the point was dynamic >
    const float beta; //!< ratio. Propability of staying dynamic given that the point was static >
    const float maxDyn; //!< ratio. Threshold for which a point will stay dynamic >
    const float maxDistNewPoint; //!< in meter. Distance at which a new point will be added in the global map >
    const float sensorMaxRange; //! <in meter. Maximum reading distance of the laser. Used to cut the global map before matching.>

public:
    ICPOdometry(ros::NodeHandle& n);
    ~ICPOdometry();

protected:
    void gotCloud(const sensor_msgs::PointCloud2& cloudMsgIn);
    void loadExternalParameters();
};

ICPOdometry::ICPOdometry(ros::NodeHandle& n):
n(n),
deltaT(PM::TransformationParameters::Identity(4, 4)),
inputQueueSize(getParam<int>("inputQueueSize", 10)),
priorDyn(getParam<double>("priorDyn", 0.5)),
priorStatic(1. - priorDyn),
maxAngle(getParam<double>("maxAngle", 0.02)),
eps_a(getParam<double>("eps_a", 0.05)),
eps_d(getParam<double>("eps_d", 0.02)),
alpha(getParam<double>("alpha", 0.99)),
beta(getParam<double>("beta", 0.99)),
maxDyn(getParam<double>("maxDyn", 0.95)),
maxDistNewPoint(pow(getParam<double>("maxDistNewPoint", 0.1),2)),
sensorMaxRange(getParam<double>("sensorMaxRange", 80.0)){

    loadExternalParameters();
    PM::Parameters params;
    params["dim"] = "-1";
    params["maxDist"] = toParam(sensorMaxRange);

    radiusFilter = PM::get().DataPointsFilterRegistrar.create("MaxDistDataPointsFilter", params);

    cloudSub = n.subscribe("cloud_in", inputQueueSize, &ICPOdometry::gotCloud, this);

    posePub = n.advertise<geometry_msgs::PoseStamped>("icp_pose", 50, true);
    pathPub = n.advertise<nav_msgs::Path>("icp_path", 50, true);

}

void ICPOdometry::gotCloud(const sensor_msgs::PointCloud2& cloudMsgIn) {
    ROS_INFO_STREAM("Received Message");
}

void ICPOdometry::loadExternalParameters() {
    string configFileName;
    if(ros::param::get("~icpConfig", configFileName)) {
        ifstream ifs(configFileName.c_str());
        if (ifs.good()) {
            icp.loadFromYaml(ifs);
        } else {
            ROS_WARN_STREAM("Cannot load ICP config from YAML file " << configFileName << ", Using default");
            icp.setDefault();
        }
    } else {
        ROS_WARN_STREAM("No ICP config file given, using default");
        icp.setDefault();
    }

    if(ros::param::get("~inputFiltersConfig", configFileName)) {
        ifstream ifs(configFileName.c_str());
        if(ifs.good()) {
            inputFilters = PM::DataPointsFilters(ifs);
        } else {
            ROS_ERROR_STREAM("Cannot load input filters config from YAML file " << configFileName);
        }
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "ICPOdometry");
    ros::NodeHandle n;
    ICPOdometry icpodometry(n);
    ros::spin();
    return 0;
}