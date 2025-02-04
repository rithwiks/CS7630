#include <math.h>
#include "TaskGoTo.h"
using namespace task_manager_msgs;
using namespace task_manager_lib;
using namespace floor_nav;

// #define DEBUG_GOTO
#ifdef DEBUG_GOTO
#warning Debugging task GOTO
#endif


TaskIndicator TaskGoTo::initialise() 
{
    RCLCPP_INFO(getNode()->get_logger(),"Going to %.2f %.2f",cfg->goal_x,cfg->goal_y);
    if (cfg->relative) {
        const geometry_msgs::msg::Pose2D & tpose = env->getPose2D();
        x_init = tpose.x;
        y_init = tpose.y;
    } else {
        x_init = 0.0;
        y_init = 0.0;
    }
    return TaskStatus::TASK_INITIALISED;
}


TaskIndicator TaskGoTo::iterate()
{
    const geometry_msgs::msg::Pose2D & tpose = env->getPose2D();
    double r = hypot(y_init + cfg->goal_y-tpose.y,x_init + cfg->goal_x-tpose.x);
    if (r < cfg->dist_threshold) {
		return TaskStatus::TASK_COMPLETED;
    }
    double alpha = remainder(atan2((y_init + cfg->goal_y-tpose.y),x_init + cfg->goal_x-tpose.x)-tpose.theta,2*M_PI);
    if (cfg->holonomic_mode) {
        double vel_x = cfg->k_v * (x_init + cfg->goal_x - tpose.x);
        double vel_y = cfg->k_v * (y_init + cfg->goal_y - tpose.y);
        
        double total_vel = hypot(vel_x, vel_y);
        if (total_vel > cfg->max_velocity) {
            vel_x *= cfg->max_velocity / total_vel;
            vel_y *= cfg->max_velocity / total_vel;
        }
        
        double rot = std::max(std::min(cfg->k_alpha * alpha, cfg->max_angular_velocity), -cfg->max_angular_velocity);
        
        env->publishVelocity(vel_x, vel_y, rot);
    } else {
#ifdef DEBUG_GOTO
        printf("c %.1f %.1f %.1f g %.1f %.1f r %.3f alpha %.1f\n",
                tpose.x, tpose.y, tpose.theta*180./M_PI,
                cfg->goal_x,cfg->goal_y,r,alpha*180./M_PI);
#endif
        if (fabs(alpha) > M_PI/9) {
            double rot = ((alpha>0)?+1:-1)*cfg->max_angular_velocity;
#ifdef DEBUG_GOTO
            printf("Cmd v %.2f r %.2f\n",0.,rot);
#endif
            env->publishVelocity(0,rot);
        } else {
            double vel = cfg->k_v * r;
            double rot = std::max(std::min(cfg->k_alpha*alpha,cfg->max_angular_velocity),-cfg->max_angular_velocity);
            if (vel > cfg->max_velocity) vel = cfg->max_velocity;
            if (vel <-cfg->max_velocity) vel = -cfg->max_velocity;
            if (rot > cfg->max_angular_velocity) rot = cfg->max_angular_velocity;
            if (rot <-cfg->max_angular_velocity) rot = -cfg->max_angular_velocity;
#ifdef DEBUG_GOTO
            printf("Cmd v %.2f r %.2f\n",vel,rot);
#endif
            env->publishVelocity(vel, rot);
        }
    }
	return TaskStatus::TASK_RUNNING;
}

TaskIndicator TaskGoTo::terminate()
{
    env->publishVelocity(0,0);
	return TaskStatus::TASK_TERMINATED;
}

DYNAMIC_TASK(TaskFactoryGoTo);
