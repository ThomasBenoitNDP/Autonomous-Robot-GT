#include <math.h>
#include "TaskStareAtFace.h"
#include "floor_nav/TaskStareAtFaceConfig.h"
using namespace task_manager_msgs;
using namespace task_manager_lib;
using namespace floor_nav;

// #define DEBUG_GOTO

TaskIndicator TaskStareAtFace::initialise() 
{
	env->publishVelocity(0,0);
    ROS_INFO("Setting heading to face detected");
    return TaskStatus::TASK_INITIALISED;
}

TaskIndicator TaskStareAtFace::iterate()
{
	const face_detect::RegionOfInterestArray & ROIl = env->getROI();
	const geometry_msgs::Pose2D & tpose = env->getPose2D();
	double xROI = ROIl.ROIlist[0].x_offset;
	double widthROI = ROIl.ROIlist[0].width;
	double xCenterROI = xROI + widthROI/2;
	double xCenterCamera = round(255./2.);
	double alpha;
	
	printf("xCenterROI %.2f xROI %.2f widthROI %.2f\n, ", xCenterROI, xROI, widthROI);
	
	if (xCenterROI > xCenterCamera) { alpha = -1;}
    else { alpha = +1;}
  
    if (xCenterROI > xCenterCamera -2 && xCenterROI < xCenterCamera +2) {
		return TaskStatus::TASK_COMPLETED;
    }
    double rot = cfg.k_theta*alpha;
    env->publishVelocity(0.0, rot);
	return TaskStatus::TASK_RUNNING;
}

TaskIndicator TaskStareAtFace::terminate()
{
    env->publishVelocity(0,0);
	return TaskStatus::TASK_TERMINATED;
}

DYNAMIC_TASK(TaskFactoryStareAtFace);
