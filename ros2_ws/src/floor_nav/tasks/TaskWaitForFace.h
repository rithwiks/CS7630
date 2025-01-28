#ifndef TASK_WAIT_FOR_ROI_H
#define TASK_WAIT_FOR_ROI_H

#include "task_manager_lib/TaskInstance.h"
#include "floor_nav/SimTasksEnv.h"
#include "cs7630_msgs/msg/region_of_interest_array.hpp"


using namespace task_manager_lib;

namespace floor_nav {
    struct TaskWaitForFaceConfig : public TaskConfig {
        TaskWaitForFaceConfig() {
            define("num_faces",  0,"max number of faces to ignore",false, num_faces);
        }

        // convenience aliases, updated by update from the config data
        int num_faces;

    };

    class TaskWaitForFace : public TaskInstance<TaskWaitForFaceConfig,SimTasksEnv>
    {
        protected:
            rclcpp::Subscription<cs7630_msgs::msg::RegionOfInterestArray>::SharedPtr faceSub;
            cs7630_msgs::msg::RegionOfInterestArray rois;
            void callback(cs7630_msgs::msg::RegionOfInterestArray::SharedPtr msg);
        public:
            TaskWaitForFace(TaskDefinitionPtr def, TaskEnvironmentPtr env) : Parent(def,env) {}
            virtual ~TaskWaitForFace() {};

            virtual TaskIndicator initialise() ;

            virtual TaskIndicator iterate();

    };
    class TaskFactoryWaitForFace : public TaskDefinition<TaskWaitForFaceConfig, SimTasksEnv, TaskWaitForFace>
    {
        public:
            TaskFactoryWaitForFace(TaskEnvironmentPtr env) : 
                Parent("WaitForFace","Do nothing until a face is detected",true,env) {}
            virtual ~TaskFactoryWaitForFace() {};
    };
}

#endif // TASK_WAIT_FOR_ROI_H
