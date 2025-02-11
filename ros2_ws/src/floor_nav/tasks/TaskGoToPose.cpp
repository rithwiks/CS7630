#include <math.h>
#include "TaskGoToPose.h"
using namespace task_manager_msgs;
using namespace task_manager_lib;
using namespace floor_nav;

// #define DEBUG_GOTO_POSE
#ifdef DEBUG_GOTO_POSE
#warning Debugging task GOTO_POSE
#endif

TaskIndicator TaskGoToPose::initialise() 
{
    RCLCPP_INFO(getNode()->get_logger(), "Going to pose (%.2f, %.2f, %.2f) using %s approach", cfg->goal_x, cfg->goal_y, cfg->goal_theta,cfg->smart_motion ? "smart" : "dumb");
    if (cfg->relative) {
        const geometry_msgs::msg::Pose2D & tpose = env->getPose2D();
        x_init = tpose.x;
        y_init = tpose.y;
        theta_init = tpose.theta;
    } else {
        x_init = 0.0;
        y_init = 0.0;
        theta_init = 0.0;
    }
    position_reached = false;
    return TaskStatus::TASK_INITIALISED;
}

TaskIndicator TaskGoToPose::iterate()
{
    if (cfg->holonomic_mode) {
        return iterateHolonomic();
    } else if (cfg->smart_motion) {
        return iterateSmart();
    } else {
        return iterateDumb();
    }
}
TaskIndicator TaskGoToPose::iterateHolonomic()
{
    const geometry_msgs::msg::Pose2D & tpose = env->getPose2D();
    
    double dx = x_init + cfg->goal_x - tpose.x;
    double dy = y_init + cfg->goal_y - tpose.y;
    double r = hypot(dy, dx);
    
    if (r < cfg->dist_threshold) {
        double theta_error = remainder(theta_init + cfg->goal_theta - tpose.theta, 2*M_PI);
        if (fabs(theta_error) < cfg->angle_threshold) {
            return TaskStatus::TASK_COMPLETED;
        }
        
        double rot = cfg->k_beta * theta_error;
        rot = std::min(std::max(rot, -cfg->max_angular_velocity), cfg->max_angular_velocity);
        env->publishVelocity(0, 0, rot);
        return TaskStatus::TASK_RUNNING;
    }
    
    double goal_angle = atan2(dy, dx);
    double theta_error = remainder(goal_angle - tpose.theta, 2*M_PI);
    
    double v_forward = cfg->k_v * r * cos(theta_error);
    double v_sideways = cfg->k_v * r * sin(theta_error);
    double omega = cfg->k_alpha * theta_error;
    
    v_forward = std::min(std::max(v_forward, -cfg->max_velocity), cfg->max_velocity);
    v_sideways = std::min(std::max(v_sideways, -cfg->max_velocity), cfg->max_velocity);
    omega = std::min(std::max(omega, -cfg->max_angular_velocity), cfg->max_angular_velocity);
    
    env->publishVelocity(v_forward, v_sideways, omega);
    
    return TaskStatus::TASK_RUNNING;
}

TaskIndicator TaskGoToPose::iterateDumb()
{
    const geometry_msgs::msg::Pose2D & tpose = env->getPose2D();
    double r = hypot(y_init + cfg->goal_y - tpose.y, x_init + cfg->goal_x - tpose.x);
    if (!position_reached) {
        if (r < cfg->dist_threshold) {
            position_reached = true;
        }
        double alpha = remainder(atan2(y_init + cfg->goal_y - tpose.y, 
                                     x_init + cfg->goal_x - tpose.x) - tpose.theta, 2*M_PI);

#ifdef DEBUG_GOTO_POSE
        printf("Position phase: c %.1f %.1f %.1f g %.1f %.1f r %.3f alpha %.1f\n",
                tpose.x, tpose.y, tpose.theta*180./M_PI,
                cfg->goal_x, cfg->goal_y, r, alpha*180./M_PI);
#endif

        if (fabs(alpha) > M_PI/9) {
            double rot = ((alpha>0)?+1:-1)*cfg->max_angular_velocity;
            env->publishVelocity(0, rot);
        } else {
            double vel = cfg->k_v * r;
            double rot = cfg->k_alpha * alpha;
            
            vel = std::min(std::max(vel, -cfg->max_velocity), cfg->max_velocity);
            rot = std::min(std::max(rot, -cfg->max_angular_velocity), cfg->max_angular_velocity);
            
            env->publishVelocity(vel, rot);
        }
    } else {
        double theta_error = remainder(theta_init + cfg->goal_theta - tpose.theta, 2*M_PI);
        
        if (fabs(theta_error) < cfg->angle_threshold) {
            return TaskStatus::TASK_COMPLETED;
        }

#ifdef DEBUG_GOTO_POSE
        printf("Orientation phase: current %.1f goal %.1f error %.1f\n",
                tpose.theta*180./M_PI, cfg->goal_theta*180./M_PI, theta_error*180./M_PI);
#endif
        double rot = cfg->k_beta * theta_error;
        rot = std::min(std::max(rot, -cfg->max_angular_velocity), cfg->max_angular_velocity);
        env->publishVelocity(0, rot);
    }
    
    return TaskStatus::TASK_RUNNING;
}

TaskIndicator TaskGoToPose::iterateSmart()
{
    const geometry_msgs::msg::Pose2D & tpose = env->getPose2D();
    double dx = x_init + cfg->goal_x - tpose.x;
    double dy = y_init + cfg->goal_y - tpose.y;
    double r = hypot(dy, dx);
    if (r < cfg->dist_threshold) {
        double theta_error = remainder(theta_init + cfg->goal_theta - tpose.theta, 2*M_PI);
        if (fabs(theta_error) < cfg->angle_threshold) {
            return TaskStatus::TASK_COMPLETED;
        }
        double rot = cfg->k_beta * theta_error;
        rot = std::min(std::max(rot, -cfg->max_angular_velocity), cfg->max_angular_velocity);
        env->publishVelocity(0, rot);
        return TaskStatus::TASK_RUNNING;
    }
    double alpha = remainder(atan2(dy, dx) - tpose.theta, 2*M_PI);  // angle to goal
    double beta = remainder(theta_init + cfg->goal_theta - tpose.theta - alpha, 2*M_PI);  // final orientation relative to goal angle

#ifdef DEBUG_GOTO_POSE
    printf("Smart control: r %.2f alpha %.1f beta %.1f\n",
            r, alpha*180./M_PI, beta*180./M_PI);
#endif
    double k_r = cfg->k_v;
    double k_a = cfg->k_alpha;
    double k_b = cfg->k_beta / 2.0;

    double v = k_r * r * cos(alpha);
    double omega = k_a * alpha + k_r * sin(alpha) * cos(alpha) / alpha * (alpha + k_b * beta);

    v = std::min(std::max(v, -cfg->max_velocity), cfg->max_velocity);
    omega = std::min(std::max(omega, -cfg->max_angular_velocity), cfg->max_angular_velocity);

    env->publishVelocity(v, omega);
    return TaskStatus::TASK_RUNNING;
}

TaskIndicator TaskGoToPose::terminate()
{
    env->publishVelocity(0, 0);
    return TaskStatus::TASK_TERMINATED;
}

DYNAMIC_TASK(TaskFactoryGoToPose);