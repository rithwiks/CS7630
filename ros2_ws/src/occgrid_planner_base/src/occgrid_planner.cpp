
#include <vector>
#include <string>
#include <map>
#include <list>
#include <chrono>
#include <functional>


#include <rclcpp/rclcpp.hpp>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#define FREE 0xFF
#define UNKNOWN 0x80
#define OCCUPIED 0x00
#define WIN_SIZE 800
# define M_PI 3.14159265358979323846

using std::placeholders::_1;
using namespace std::chrono_literals;

class OccupancyGridPlanner : public rclcpp::Node {
    protected:
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr og_sub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
        rclcpp::TimerBase::SharedPtr timer_;


        std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_buffer;

        cv::Rect roi_;
        std::vector<cv::Mat_<uint8_t>> og_, cropped_og_;
        cv::Mat_<cv::Vec3b> og_rgb_, og_rgb_marked_;
        cv::Point og_center_;
        nav_msgs::msg::MapMetaData info_;
        std::string frame_id_;
        std::string base_link_;
        bool headless_;
        bool ready_;
        bool debug_;
        double robot_radius_;

        typedef std::multimap<float, cv::Point3i> Heap;

        // Example of code to convert between Point3i and Point2i, aka Point
        cv::Point P2(const cv::Point3i & P) {return cv::Point(P.x,P.y);}

        // Callback for Occupancy Grids
        void og_callback(nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
            info_ = msg->info;
            frame_id_ = msg->header.frame_id;
            // Create an image to store the value of the grid.
            og_.resize(8);
            cropped_og_.resize(8);
            for (int i = 0; i < 8; i++) {
                og_[i] = cv::Mat_<uint8_t>(msg->info.height, msg->info.width, 0xFF);
            }
            og_center_ = cv::Point(-info_.origin.position.x/info_.resolution,
                    -info_.origin.position.y/info_.resolution);

            // Some variables to select the useful bounding box 
            unsigned int maxx=0, minx=msg->info.width, 
                         maxy=0, miny=msg->info.height;
            // Convert the representation into something easy to display.
            for (unsigned int j=0;j<msg->info.height;j++) {
                for (unsigned int i=0;i<msg->info.width;i++) {
                    for (int k = 0; k < 8; k++) {
                        int8_t v = msg->data[j*msg->info.width + i];
                        switch (v) {
                            case 0: 
                                og_[k].at<unsigned char>(j,i) = FREE; 
                                break;
                            case 100: 
                                og_[k].at<unsigned char>(j,i) = OCCUPIED; 
                                break;
                            case -1: 
                            default:
                                og_[k].at<unsigned char>(j,i) = UNKNOWN; 
                                break;
                        }
                        // Update the bounding box of free or occupied cells.
                        if (og_[k].at<unsigned char>(j,i) != UNKNOWN) {
                            minx = std::min(minx,i);
                            miny = std::min(miny,j);
                            maxx = std::max(maxx,i);
                            maxy = std::max(maxy,j);
                        }
                    }
                }
            }
            // TODO: Implement obstacle expansion here
            // -----------------------
            double erosion_size = robot_radius_ / info_.resolution;
            cv::Mat element = getStructuringElement(cv::MORPH_RECT, cv::Size(2*erosion_size+1, 2*erosion_size+1), cv::Point(erosion_size, erosion_size));
            for (int i = 0; i < 8; i++) {
                cv::erode(og_[i], og_[i], element);
            }
            
            if (!ready_) {
                ready_ = true;
                RCLCPP_INFO(this->get_logger(),"Received occupancy grid, ready_ to plan");
            }
            // The lines below are only for display
            unsigned int w = maxx - minx;
            unsigned int h = maxy - miny;
            roi_ = cv::Rect(minx,miny,w,h);
            if (!headless_) {
                for (int i = 0; i < 8; i++) {
                    cv::cvtColor(og_[i], og_rgb_, cv::COLOR_GRAY2RGB);
                    // Compute a sub-image that covers only the useful part of the
                    // grid.
                    
                    cropped_og_[i] = cv::Mat_<uint8_t>(og_[i],roi_);
                    
                    
                    if ((w > WIN_SIZE) || (h > WIN_SIZE)) {
                        // The occupancy grid is too large to display. We need to scale
                        // it first.
                        double ratio = w / ((double)h);
                        cv::Size new_size;
                        if (ratio >= 1) {
                            new_size = cv::Size(WIN_SIZE,WIN_SIZE/ratio);
                        } else {
                            new_size = cv::Size(WIN_SIZE*ratio,WIN_SIZE);
                        }
                        cv::Mat_<uint8_t> resized_og;
                        cv::resize(cropped_og_[i],resized_og,new_size);
                        cv::imshow( "OccGrid", resized_og );
                    } else {
                        // cv::imshow( "OccGrid", cropped_og_ );
                        cv::imshow( "OccGrid", og_rgb_ );
                    }
                }
            }
        }

        // Generic test if a point is within the occupancy grid
        bool isInGrid(const cv::Point & P) {
            if ((P.x < 0) || (P.x >= (signed)info_.width) 
                    || (P.y < 0) || (P.y >= (signed)info_.height)) {
                return false;
            }
            return true;
        }

        // This is called when a new goal is posted by RViz. We don't use a
        // mutex here, because it can only be called in spinOnce.
        void target_callback(geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            geometry_msgs::msg::TransformStamped transformStamped;
            geometry_msgs::msg::PoseStamped pose;
            if (!ready_) {
                RCLCPP_WARN(this->get_logger(),"Ignoring target while the occupancy grid has not been received");
                return;
            }
            RCLCPP_INFO(this->get_logger(),"Received planning request in frame %s (base link %s, frame id %s)",
                    msg->header.frame_id.c_str(),
                    base_link_.c_str(),
                    frame_id_.c_str());
            og_rgb_marked_ = og_rgb_.clone();
            // Convert the destination point in the occupancy grid frame. 
            // The debug case is useful is the map is published without
            // gmapping running (for instance with map_server).
            if (debug_) {
                pose = *msg;
            } else {
                try{
                    std::string errStr;
                    // This converts target in the grid frame.
                    if (!tf_buffer->canTransform(frame_id_, msg->header.frame_id, msg->header.stamp,
                                rclcpp::Duration(std::chrono::duration<double>(1.0)),&errStr)) {
                        RCLCPP_ERROR(this->get_logger(),"Cannot transform target: %s",errStr.c_str());
                        return;
                    }
                    transformStamped = tf_buffer->lookupTransform(frame_id_, msg->header.frame_id, msg->header.stamp);
                    tf2::doTransform(*msg,pose,transformStamped);

                    // this gets the current pose in transform
                    if (!tf_buffer->canTransform(frame_id_, base_link_, msg->header.stamp,
                                rclcpp::Duration(std::chrono::duration<double>(1.0)),&errStr)) {
                        RCLCPP_ERROR(this->get_logger(),"Cannot transform base_link: %s",errStr.c_str());
                        return;
                    }
                    transformStamped = tf_buffer->lookupTransform(frame_id_, base_link_, tf2::TimePointZero);
                }
                catch (const tf2::TransformException & ex){
                    RCLCPP_ERROR(this->get_logger(),"%s",ex.what());
                }
            }
            // Now scale the target to the grid resolution and shift it to the
            // grid center.
            // For reference, this recovers the robot orientation
            double t_yaw = tf2::getYaw(pose.pose.orientation);
            cv::Point target2d = cv::Point(pose.pose.position.x / info_.resolution, 
                    pose.pose.position.y / info_.resolution)
                + og_center_;
            cv::Point3i target = cv::Point3i(target2d.x, target2d.y, int(round(t_yaw / (M_PI / 4))) < 0 ? int(round(t_yaw / (M_PI / 4))) + 8 : int(round(t_yaw / (M_PI / 4))));
            RCLCPP_INFO(this->get_logger(),"Planning target: %.2f %.2f %.2f-> %d %d %d",
                    pose.pose.position.x, pose.pose.position.y, t_yaw, target.x, target.y, target.z);
            cv::circle(og_rgb_marked_,target2d, 10, cv::Scalar(0,0,255));
            if (!headless_) {
                cv::imshow( "OccGrid", og_rgb_marked_ );
            }
            if (!isInGrid(target2d)) {
                RCLCPP_ERROR(this->get_logger(),"Invalid target point (%.2f %.2f) -> (%d %d)",
                        pose.pose.position.x, pose.pose.position.y, target.x, target.y);
                return;
            }
            // Only accept target which are FREE in the grid (HW, Step 5).
            og_[target.z].at<unsigned char>(target.y, target.x);
    
            
            if (og_[target.z].at<unsigned char>(target.y, target.x) != FREE) {
                RCLCPP_ERROR(this->get_logger(),"Invalid target point: occupancy = %d",og_[target.z].at<unsigned char>(target.y, target.x));
                return;
            }

            // Now get the current point in grid coordinates.
            cv::Point start2d;
            double s_yaw = 0;
            if (debug_) {
                start2d = og_center_;
                s_yaw = 0;
            } else {
                // For reference, this is how we get the current pose orientation
                s_yaw = tf2::getYaw(transformStamped.transform.rotation);
                start2d = cv::Point(transformStamped.transform.translation.x / info_.resolution, 
                        transformStamped.transform.translation.y / info_.resolution)
                    + og_center_;
            }
            RCLCPP_INFO(this->get_logger(),"Planning origin %.2f %.2f %.2f -> %d %d",
                    transformStamped.transform.translation.x, transformStamped.transform.translation.y,
                    s_yaw, start2d.x, start2d.y);
            cv::circle(og_rgb_marked_,start2d, 10, cv::Scalar(0,255,0));
            if (!headless_) {
                cv::imshow( "OccGrid", og_rgb_marked_ );
            }
            if (!isInGrid(start2d)) {
                RCLCPP_ERROR(this->get_logger(),"Invalid starting point (%.2f %.2f) -> (%d %d)",
                        transformStamped.transform.translation.x, transformStamped.transform.translation.y,
                        start2d.x, start2d.y);
                return;
            }
            // If the starting point is not FREE there is a bug somewhere, but
            // better to check
            cv::Point3i start = cv::Point3i(start2d.x, start2d.y, int(round(s_yaw / (M_PI / 4))) < 0 ? int(round(s_yaw / (M_PI / 4))) + 8 : int(round(s_yaw / (M_PI / 4))));
            RCLCPP_ERROR(this->get_logger(),"here %d, %d, %d", start.x, start.y, start.z);
            if (og_[start.z].at<unsigned char>(start.y, start.x) != FREE) {
                RCLCPP_ERROR(this->get_logger(),"Invalid start point: occupancy = %d",og_[start.z].at<unsigned char>(start.y, start.x));
                return;
            }
            
            RCLCPP_INFO(this->get_logger(),"Starting planning from (%d, %d,%d) to (%d, %d,%d)",start.x,start.y, start.z, target.x, target.y, target.z);
            // Here the Dijskstra algorithm starts 
            // The best distance to the goal computed so far. This is
            // initialised with Not-A-Number. 
            // cv::Mat_<float> cell_value(og_.size(), NAN);
            // For each cell we need to store a pointer to the coordinates of
            // its best predecessor. 
            // cv::Mat_<cv::Vec2s> predecessor(og_.size());
            
            // TODO: For reference, this is how we would make the arrays 3D
            int dims[3] = {og_[0].size().width, og_[0].size().height, 8};
            cv::Mat_<float> cell_value(3,dims, NAN);
            cv::Mat_<cv::Vec3s> predecessor(3,dims);

            // The neighbour of a given cell in relative coordinates. The order
            // is important. If we use 4-connexity, then we can use only the
            // first 4 values of the array. If we use 8-connexity we use the
            // full array.
            cv::Point3i neighbours[8][7] = {{cv::Point3i(1,0,0),cv::Point3i(1,1,2),cv::Point3i(1,-1,-2),cv::Point3i(2,1,1),cv::Point3i(2,-1,-1),cv::Point3i(0,0,1), cv::Point3i(0,0,-1)},
                            {cv::Point3i(1,1,0),cv::Point3i(0,1,2),cv::Point3i(1,0,-2),cv::Point3i(1,2,1),cv::Point3i(2,1,-1),cv::Point3i(0,0,1), cv::Point3i(0,0,-1)},
                            {cv::Point3i(0,1,0),cv::Point3i(-1,1,2),cv::Point3i(1,1,-2),cv::Point3i(-1,2,1),cv::Point3i(1,2,-1),cv::Point3i(0,0,1), cv::Point3i(0,0,-1)},
                            {cv::Point3i(-1,1,0),cv::Point3i(-1,0,2),cv::Point3i(0,1,-2),cv::Point3i(-2,1,1),cv::Point3i(-1,2,-1),cv::Point3i(0,0,1), cv::Point3i(0,0,-1)},
                            {cv::Point3i(-1,0,0),cv::Point3i(-1,-1,2),cv::Point3i(-1,1,-2),cv::Point3i(-2,-1,1),cv::Point3i(-2,1,-1),cv::Point3i(0,0,1), cv::Point3i(0,0,-1)},
                            {cv::Point3i(-1,-1,0),cv::Point3i(0,-1,2),cv::Point3i(-1,0,-2),cv::Point3i(-1,-2,1),cv::Point3i(-2,-1,-1),cv::Point3i(0,0,1), cv::Point3i(0,0,-1)},
                            {cv::Point3i(0,-1,0),cv::Point3i(1,-1,2),cv::Point3i(-1,-1,-2),cv::Point3i(1,-2,1),cv::Point3i(-1,-2,-1),cv::Point3i(0,0,1), cv::Point3i(0,0,-1)},
                            {cv::Point3i(1,-1,0),cv::Point3i(1,0,2),cv::Point3i(0,-1,-2),cv::Point3i(2,-1,1),cv::Point3i(1,-2,-1),cv::Point3i(0,0,1), cv::Point3i(0,0,-1)}
                                    };
            // TODO: Create a new set of neighbours in 3D
            // std::array<cv::Point3i,1> neighbours = {cv::Point3i(0,0,0)}; 
            // Cost of displacement corresponding the neighbours. Diagonal
            // moves are 44% longer.
            float cost[2][7] = {{1, sqrt(2), sqrt(2), sqrt(2), sqrt(2), .5, .5}, {sqrt(2), 1, 1, sqrt(3), sqrt(3), .5, .5}};
            
            // The core of Dijkstra's Algorithm, a sorted heap, where the first
            // element is always the closer to the start.
            // TODO: from Dijkstra to A*, add a heuristic and an early exit
            Heap heap;
            heap.insert(Heap::value_type(0, start));
            cell_value(start.x,start.y, start.z) = 0;
            while (!heap.empty()) {
                // Select the cell at the top of the heap
                Heap::iterator hit = heap.begin();
                // the cell it contains is this_cell
                cv::Point3i this_cell = hit->second;
                // and its score is this_cost
                float this_cost = cell_value(this_cell.x,this_cell.y, this_cell.z);
                // We can remove it from the heap now.
                heap.erase(hit);
                if (this_cell.x == target.x && this_cell.y == target.y && this_cell.z == target.z) {
                    RCLCPP_INFO(this->get_logger(),"Early stop");
                    break;
                }
                // Now see where we can go from this_cell
                for (size_t i=0;i<7;i++) {
                    cv::Point3i dest = this_cell + neighbours[this_cell.z][i];
                    if (!isInGrid(P2(dest))) {
                        // outside the grid
                        continue;
                    }
                    dest.z = dest.z < 0 ? dest.z + 8 : dest.z;
                    dest.z %= 8;
                    uint8_t og = og_[dest.z].at<unsigned char>(dest.y, dest.x);
                    if (og != FREE) {
                        // occupied or unknown
                        continue;
                    }
                    RCLCPP_INFO(this->get_logger(),"%d, %d,%d from %d, %d,%d",dest.x,dest.y, dest.z, this_cell.x, this_cell.y, this_cell.z);
                    float cv = cell_value(dest.x,dest.y, dest.z);
                    float new_cost = this_cost + cost[this_cell.z%2][i];
                    if (std::isnan(cv) || (new_cost < cv)) {
                        // found shortest path (or new path), updating the
                        // predecessor and the value of the cell
                        predecessor.at<cv::Vec3s>(dest.x,dest.y,dest.z) = cv::Vec3s(this_cell.x,this_cell.y,this_cell.z);
                        cell_value(dest.x,dest.y,dest.z) = new_cost;
                        // And insert the selected cells in the map.
                        double ang_cost = (std::min(fabs(target.z - dest.z), 8 - fabs(target.z - dest.z))) / 4.0;
                        double heur = sqrt((target.x - dest.x) * (target.x - dest.x) + (target.y - dest.y) * (target.y - dest.y)) +  ang_cost;
                        heap.insert(Heap::value_type(new_cost + heur, dest));
                    }
                }
            }
            if (isnan(cell_value(target.x,target.y, target.z))) {
                // No path found
                RCLCPP_ERROR(this->get_logger(),"No path found from (%d, %d) to (%d, %d)",
                        start.x,start.y,target.x,target.y);
                return;
            }
            RCLCPP_INFO(this->get_logger(),"Planning completed");
            // Now extract the path by starting from goal and going through the
            // predecessors until the starting point
            std::list<cv::Point3i> lpath;
            while (target != start) {
                assert(lpath.size()<1000000);
                lpath.push_front(target);
                cv::Vec3s p = predecessor(target.x,target.y,target.z);
                target.x = p[0]; target.y = p[1]; target.z = p[2];
            }
            lpath.push_front(start);
            // Finally create a ROS path message
            nav_msgs::msg::Path path;
            path.header.stamp = this->get_clock()->now();
            path.header.frame_id = frame_id_;
            path.poses.resize(lpath.size());
            std::list<cv::Point3i>::const_iterator it = lpath.begin();
            unsigned int ipose = 0;
            while (it != lpath.end()) {
                // time stamp is not updated because we're not creating a
                // trajectory at this stage
                path.poses[ipose].header = path.header;
                cv::Point P = P2(*it) - og_center_;
                path.poses[ipose].pose.position.x = (P.x) * info_.resolution;
                path.poses[ipose].pose.position.y = (P.y) * info_.resolution;
                tf2::Quaternion Q;
                double angle = it->z * M_PI/4;
                angle = angle > M_PI / 2 ? M_PI - angle : angle;
                Q.setRPY(0,0,it->z * M_PI/4);
                path.poses[ipose].pose.orientation = tf2::toMsg(Q);
                ipose++;
                it ++;
            }
            path_pub_->publish(path);
            RCLCPP_INFO(this->get_logger(),"Request completed");
        }



    public:
        OccupancyGridPlanner() : rclcpp::Node("occgrid_planner") {
            ready_ = false;
            this->declare_parameter("~/base_frame",std::string("body"));
            this->declare_parameter("~/debug",false);
            this->declare_parameter("~/headless",true);
            this->declare_parameter("~/robot_radius",0.1);
            base_link_ = this->get_parameter("~/base_frame").as_string();
            debug_ = this->get_parameter("~/debug").as_bool();
            headless_ = this->get_parameter("~/headless").as_bool();
            robot_radius_ = this->get_parameter("~/robot_radius").as_double();
            tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
            og_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("~/occ_grid",1,
                    std::bind(&OccupancyGridPlanner::og_callback,this,std::placeholders::_1));
            target_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("~/goal",1,
                    std::bind(&OccupancyGridPlanner::target_callback,this,std::placeholders::_1));
            path_pub_ = this->create_publisher<nav_msgs::msg::Path>("~/path",1);

            if (!headless_) {
                cv::namedWindow( "OccGrid", cv::WINDOW_AUTOSIZE );
                timer_ = this->create_wall_timer( 50ms,
                        std::bind(&OccupancyGridPlanner::timer_cb, this));
            }
        }

        void timer_cb() {
            cv::waitKey(5);
        }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OccupancyGridPlanner>());
    rclcpp::shutdown();
}

