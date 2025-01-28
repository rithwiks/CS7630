#include <math.h>
#include "TaskWaitForFace.h"
using namespace task_manager_msgs;
using namespace task_manager_lib;
using namespace floor_nav;

TaskIndicator TaskWaitForFace::initialise()
{
    faceSub = getNode()->create_subscription<cs7630_msgs::msg::RegionOfInterestArray>("/face_detect/detected_faces",1,
    std::bind(&TaskWaitForFace::callback, this, std::placeholders::_1));
    return TaskStatus::TASK_INITIALISED;
}

void TaskWaitForFace::callback(cs7630_msgs::msg::RegionOfInterestArray::SharedPtr msg) {
    rois = *msg;
}


TaskIndicator TaskWaitForFace::iterate()
{
    if (rois.regions.size() > cfg->num_faces) {
        for (int i = 0; i < rois.regions.size(); i++) {
            RCLCPP_INFO(node->get_logger(),"Detected Face at %.2d %.2d",rois.regions[i].x_offset, rois.regions[i].y_offset);
        }
		return TaskStatus::TASK_COMPLETED;
    }
	return TaskStatus::TASK_RUNNING;
}

DYNAMIC_TASK(TaskFactoryWaitForFace)
