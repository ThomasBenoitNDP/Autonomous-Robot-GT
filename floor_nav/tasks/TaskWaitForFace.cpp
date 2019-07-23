#include <math.h>
#include "TaskWaitForFace.h"
#include "floor_nav/TaskWaitForFaceConfig.h"
using namespace task_manager_msgs;
using namespace task_manager_lib;
using namespace floor_nav;



TaskIndicator TaskWaitForFace::iterate()
{	
	double size;
	const face_detect::RegionOfInterestArray & ROIl = env->getROI();
	size = ROIl.ROIlist.size();
	if (size != 0){
		return TaskStatus::TASK_COMPLETED;
	}
	
	return TaskStatus::TASK_RUNNING;
}

DYNAMIC_TASK(TaskFactoryWaitForFace);
