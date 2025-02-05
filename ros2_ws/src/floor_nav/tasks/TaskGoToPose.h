#ifndef TASK_GOTO_POSE_H
#define TASK_GOTO_POSE_H

#include "task_manager_lib/TaskInstance.h"
#include "floor_nav/SimTasksEnv.h"

using namespace task_manager_lib;

namespace floor_nav {
    struct TaskGoToPoseConfig : public TaskConfig {
        TaskGoToPoseConfig() {
            define("goal_x", 0., "X coordinate of destination", false, goal_x);
            define("goal_y", 0., "Y coordinate of destination", false, goal_y);
            define("goal_theta", 0., "Desired orientation at destination", false, goal_theta);
            define("k_v", 1.0, "Gain for velocity control", false, k_v);
            define("k_alpha", 1.0, "Gain for angular control", false, k_alpha);
            define("k_beta", 1.0, "Gain for final orientation control", false, k_beta);
            define("max_velocity", 1.0, "Max allowed velocity", false, max_velocity);
            define("max_angular_velocity", 1.0, "Max allowed angular velocity", false, max_angular_velocity);
            define("dist_threshold", 0.1, "Distance at which the target is considered reached", false, dist_threshold);
            define("angle_threshold", 0.1, "Angle at which the target orientation is considered reached", false, angle_threshold);
            define("relative", false, "Is the target pose relative or absolute", true, relative);
            define("smart_motion", true, "Use smart kinematic control instead of dumb approach", true, smart_motion);
            define("holonomic", false, "Allow holonomic movement (sideways/diagonal motion)", true, holonomic_mode);

        }

        // convenience aliases, updated by update from the config data
        double goal_x, goal_y, goal_theta;
        double k_v, k_alpha, k_beta;
        double max_velocity;
        double max_angular_velocity;
        double dist_threshold;
        double angle_threshold;
        bool relative;
        bool smart_motion;
        bool holonomic_mode;
    };

    class TaskGoToPose : public TaskInstance<TaskGoToPoseConfig, SimTasksEnv>
    {
        protected:
            double x_init, y_init, theta_init;
            bool position_reached;
            TaskIndicator iterateDumb();
            TaskIndicator iterateSmart();
            TaskIndicator iterateHolonomic();

        public:
            TaskGoToPose(TaskDefinitionPtr def, TaskEnvironmentPtr env) : Parent(def,env) {
                position_reached = false;
            }
            virtual ~TaskGoToPose() {};

            virtual TaskIndicator initialise();
            virtual TaskIndicator iterate();
            virtual TaskIndicator terminate();
    };

    class TaskFactoryGoToPose : public TaskDefinition<TaskGoToPoseConfig, SimTasksEnv, TaskGoToPose>
    {
        public:
            TaskFactoryGoToPose(TaskEnvironmentPtr env) : 
                Parent("GoToPose", "Reach a desired pose (position and orientation)", true, env) {}
            virtual ~TaskFactoryGoToPose() {};
    };
};

#endif // TASK_GOTO_POSE_H