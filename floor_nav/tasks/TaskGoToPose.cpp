#include <math.h>

#include "TaskGoToPose.h"
#include "floor_nav/TaskGoToPoseConfig.h"
using namespace task_manager_msgs;
using namespace task_manager_lib;
using namespace floor_nav;

#define DEBUG_GOTO
#ifdef DEBUG_GOTO
#warning Debugging task GOTO
#endif



TaskIndicator TaskGoToPose::initialise() 
{   
    ROS_INFO("Going to %.2f %.2f %.2f",cfg.goal_x,cfg.goal_y, cfg.goal_angle);
    if (cfg.relative) {
        const geometry_msgs::Pose2D & tpose = env->getPose2D();
        x_init = tpose.x;
        y_init = tpose.y;
	a_init = tpose.theta;
    } else {
        x_init = 0.0;
        y_init = 0.0;
	a_init = 0.0;
    }
    return TaskStatus::TASK_INITIALISED;
}


TaskIndicator TaskGoToPose::iterate()
{
	double goal_angle;
	
	if (abs(cfg.goal_angle*M_PI/180.)> 2*M_PI) goal_angle = remainder(cfg.goal_angle*M_PI/180., 2*M_PI);	
	else goal_angle = cfg.goal_angle*M_PI/180.;
	
    const geometry_msgs::Pose2D & tpose = env->getPose2D();
    double r = hypot(y_init + cfg.goal_y-tpose.y,x_init + cfg.goal_x-tpose.x);
    double cond_final;  
      
    if (cfg.smart_mode == true){
		 cond_final = fabs(goal_angle + a_init - tpose.theta);
	}else {
		 cond_final = 0.0;
	}
     
    if (r < cfg.dist_threshold) {

		if(cond_final < cfg.angle_error){
			return TaskStatus::TASK_COMPLETED;
		}else{
			double diff_angle = remainder(goal_angle + a_init - tpose.theta,2*M_PI) ;
			
			if (diff_angle > M_PI) diff_angle = diff_angle -2*M_PI;
			else if (diff_angle < -1*M_PI) diff_angle = diff_angle + 2*M_PI; 	
	
			
            double w = cfg.k_v_f*(diff_angle);
            
#ifdef DEBUG_GOTO
            printf(" diff_angle = %.2f \n",  diff_angle);
#endif
            env->publishVelocity(0,w);
            return TaskStatus::TASK_RUNNING;
		
		}
		
    }
     double alpha = remainder(atan2((y_init + cfg.goal_y-tpose.y),x_init + cfg.goal_x-tpose.x)-tpose.theta,2*M_PI);
      
	
#ifdef DEBUG_GOTO
    printf("c %.1f %.1f %.1f g %.1f %.1f r %.3f alpha %.1f\n",
            tpose.x, tpose.y, tpose.theta*180./M_PI,
            cfg.goal_x,cfg.goal_y,r,alpha*180./M_PI);
#endif
    if (fabs(alpha) > M_PI/9) {
	
        double rot = ((alpha>0)?+1:-1)*cfg.max_angular_velocity;
#ifdef DEBUG_GOTO
        printf("Cmd v %.2f r %.2f\n",0.,rot);
	printf("PI > 9  alpha = %.2f \n", alpha);
#endif
        env->publishVelocity(0,rot);
    } else {
	
        double vel = cfg.k_v * r;
        double rot = std::max(std::min(cfg.k_alpha*alpha,cfg.max_angular_velocity),-cfg.max_angular_velocity);
        if (vel > cfg.max_velocity) vel = cfg.max_velocity;
#ifdef DEBUG_GOTO
        printf("Cmd v %.2f r %.2f\n",vel,rot);
#endif
        env->publishVelocity(vel, rot);
    }
   
       return TaskStatus::TASK_RUNNING;
}



/*********************/
TaskIndicator TaskGoToPose::terminate()
{ 
#ifdef DEBUG_GOTO
    printf(" ON ST DANS LE TERMINATE  \n");
#endif
	env->publishVelocity(0,0);		
	return TaskStatus::TASK_TERMINATED;
}

DYNAMIC_TASK(TaskFactoryGoToPose);
