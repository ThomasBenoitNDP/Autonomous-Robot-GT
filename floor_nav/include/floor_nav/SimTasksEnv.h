#ifndef SIM_TASKS_ENV_H
#define SIM_TASKS_ENV_H

#include <ros/ros.h>
#include "task_manager_lib/TaskDefinition.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/LaserScan.h"
#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include "face_detect/RegionOfInterestArray.h"

namespace floor_nav {
    class SimTasksEnv: public task_manager_lib::TaskEnvironment
    {
        protected:
            bool paused;
            ros::Subscriber muxSub;
            ros::Subscriber pointCloudSub;
            ros::Subscriber pointCloud2DSub;
            ros::Subscriber laserscanSub;
            ros::Subscriber faceDetectSub;
            ros::Publisher velPub;
            ros::ServiceClient muxClient;
            tf::TransformListener listener;

            void muxCallback(const std_msgs::String::ConstPtr& msg) ;
            void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr msg) ;
            void pointCloud2DCallback(const sensor_msgs::PointCloud2ConstPtr msg) ;
            void laserScanCallback(const sensor_msgs::LaserScanConstPtr msg) ;
            void faceDetectCallback(const face_detect::RegionOfInterestArrayConstPtr msg) ;

            bool manualControl;
            std::string joystick_topic;
            std::string auto_topic;
            std::string base_frame;
            std::string reference_frame;
            pcl::PointCloud<pcl::PointXYZ> pointCloud;
            pcl::PointCloud<pcl::PointXYZ> pointCloud2D;
            face_detect::RegionOfInterestArray multiROI;
			
						
        public:
            SimTasksEnv(ros::NodeHandle & nh);
            ~SimTasksEnv() {};

            ros::NodeHandle & getNodeHandle() {return nh;}

            geometry_msgs::Pose2D getPose2D() const ; 

            geometry_msgs::Pose getPose() const ;
            
            geometry_msgs::PoseStamped getPoseStamped() const  ;
            
            
            const pcl::PointCloud<pcl::PointXYZ>& getPointCloud() const {return pointCloud;}
            const pcl::PointCloud<pcl::PointXYZ>& getPointCloud2D() const {return pointCloud2D;}
			const face_detect::RegionOfInterestArray getROI() const {return multiROI;}


            void publishVelocity(double linear, double angular) ;
            void publishVelocity(double linear_x, double linear_y, double angular) ;
            void publishVelocity(const geometry_msgs::Twist & twist) ;

            void setManualControl();
            void setComputerControl();
            bool getManualControl() const {return manualControl;}

            const std::string & getReferenceFrame() const {return reference_frame;}
            const std::string & getBaseFrame() const {return base_frame;}
        public: // To make point cloud work on 32bit system
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    };

};

#endif // SIM_TASKS_ENV_H
