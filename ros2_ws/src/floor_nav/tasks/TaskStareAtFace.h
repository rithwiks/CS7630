#ifndef TASK_SET_HEADING_H
#define TASK_SET_HEADING_H

#include "task_manager_lib/TaskInstance.h"
#include "floor_nav/SimTasksEnv.h"
#include "cs7630_msgs/msg/region_of_interest_array.hpp"

using namespace task_manager_lib;

namespace floor_nav {
    struct TaskStareAtFaceConfig : public TaskConfig {
        TaskStareAtFaceConfig() {
            define("max_angular_velocity",  1.0,"Maximum angular velocity",false,max_angular_velocity);
            define("k_theta",  1.0,"Gain for angular control",false,k_theta);
            define("pixel_threshold",  2.0 ,"Pixel error at which the target is considered reached",false, pixel_threshold);
        }

        // convenience aliases, updated by update from the config data
        double k_theta;
        double max_angular_velocity;
        double pixel_threshold;
    };

    class TaskStareAtFace : public TaskInstance<TaskStareAtFaceConfig,SimTasksEnv>
    {
        protected: 
            rclcpp::Subscription<cs7630_msgs::msg::RegionOfInterestArray>::SharedPtr faceSub;
            cs7630_msgs::msg::RegionOfInterestArray rois;
            void callback(cs7630_msgs::msg::RegionOfInterestArray::SharedPtr msg);

        public:
            TaskStareAtFace(TaskDefinitionPtr def, TaskEnvironmentPtr env) : Parent(def,env) {}
            virtual ~TaskStareAtFace() {};

            virtual TaskIndicator initialise() ;

            virtual TaskIndicator iterate();

            virtual TaskIndicator terminate();
    };
    class TaskFactoryStareAtFace : public TaskDefinition<TaskStareAtFaceConfig, SimTasksEnv, TaskStareAtFace>
    {

        public:
            TaskFactoryStareAtFace(TaskEnvironmentPtr env) : 
                Parent("StareAtFace","If a face is detected, center it",true,env) {}
            virtual ~TaskFactoryStareAtFace() {};
    };
};

#endif // TASK_SET_HEADING_H
