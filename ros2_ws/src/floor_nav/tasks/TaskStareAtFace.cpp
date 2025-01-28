#include <math.h>
#include "TaskStareAtFace.h"
using namespace task_manager_msgs;
using namespace task_manager_lib;
using namespace floor_nav;

// #define DEBUG_GOTO

TaskIndicator TaskStareAtFace::initialise() 
{
    faceSub = getNode()->create_subscription<cs7630_msgs::msg::RegionOfInterestArray>("/face_detect/detected_faces",1,
    std::bind(&TaskStareAtFace::callback, this, std::placeholders::_1));
    return TaskStatus::TASK_INITIALISED;
}

void TaskStareAtFace::callback(cs7630_msgs::msg::RegionOfInterestArray::SharedPtr msg) {
    rois = *msg;
}

TaskIndicator TaskStareAtFace::iterate()
{
    if (rois.regions.size() > 0) {
        RCLCPP_INFO(node->get_logger(),"Detected Face at %.2d %.2d with width %.2d",rois.regions[0].x_offset, rois.regions[0].y_offset, rois.regions[0].width);
        double center = rois.regions[0].x_offset + rois.regions[0].width / 2.0;
        double alpha = (128.0 - center) / 128.0;
        RCLCPP_INFO(node->get_logger(),"Detected Face at %.2f with alpha %.2f", center, alpha);
        if (fabs(alpha*128.0) < cfg->pixel_threshold) {
            return TaskStatus::TASK_COMPLETED;
        }
		double rot = cfg->k_theta*alpha;
        if (rot > cfg->max_angular_velocity) rot = cfg->max_angular_velocity;
        if (rot <-cfg->max_angular_velocity) rot =-cfg->max_angular_velocity;
        RCLCPP_INFO(node->get_logger(),"Turning with %.2f",rot);
        env->publishVelocity(0.0, rot);
        return TaskStatus::TASK_RUNNING;
    }
	return TaskStatus::TASK_RUNNING;
}

TaskIndicator TaskStareAtFace::terminate()
{
    env->publishVelocity(0,0);
	return TaskStatus::TASK_TERMINATED;
}

DYNAMIC_TASK(TaskFactoryStareAtFace);
