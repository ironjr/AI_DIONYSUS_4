// Debug Option
// #define DEBUG_TEXT
// #define DEBUG_POINT_CLOUD
// #define DEBUG_BALL
#define DEBUG_IMAGE
#define VISUALIZE_PATH false

#define SCENARIO 3

#define CONST_K 50000
#define MAX_STEP 0.25

#define LOOK_AHEAD_DISTANCE 0.5
#define GOAL_CONFIRM_DISTANCE 0.2

#define MARGIN 10

// Dynamic mapping
#define HORIZON 274 // HORIZON <= "Ground" <= 480
#define CHECK_Z_ABOVE_HORIZON 20
#define GROUND_HEIGHT 0.309900 // Kinect y axis

#define COLLISION_CHECK_RATE 3 // # of iterations passed before next collision check
// #define KINECT_MARGIN_W 10
#define KINECT_VIEWAREA_X 1.1
#define CAMERA_SAMPLING_RATE_H 3
#define CAMERA_SAMPLING_RATE_W 3

#define OBSTACLE_DETERMINENT_DISTANCE 3
#define OBSTACLE_CONFIRM_H 1
#define OBSTACLE_CONFIRM_W 1
#define OBSTACLE_CONFIRM_THRESHOLD 5
#define OBSTACLE_HEIGHT_MIN 0.1
#define OBSTACLE_HEIGHT_MAX 0.35

#define CLEARANCE_DETERMINENT_DISTANCE 2.5
#define CLEARANCE_CONFIRM_H 3
#define CLEARANCE_CONFIRM_W 3
#define CLEARANCE_CONFIRM_THRESHOLD 5
#define CLEARANCE_HEIGHT_LOW 0.05
#define CLEARANCE_HEIGHT_HIGH 0.45

#define SLEEP_SETTLING_TIME 5

#define NEW_OBSTACLE_MARGIN 10
#define NEW_CLEARANCE_MARGIN 10
#define NEW_MARGIN_TOLERANCE 13

// State definition
#define INIT 0
#define RUNNING 1
#define MAPPING 2
#define PATH_PLANNING 3
#define FINISH -1


#include <unistd.h>
#include <ros/ros.h>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/ModelStates.h>
#include <project4/purePursuit.h>
#include <project4/rrtTree.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/visualization/cloud_viewer.h>
#include <algorithm>


// Debug image
#ifdef DEBUG_IMAGE
    static int imageNumberMain = 1;
#endif

// Map spec
cv::Mat map;
cv::Mat dynamic_map;
double res;
int map_y_range;
int map_x_range;
double map_origin_x;
double map_origin_y;
double world_x_min;
double world_x_max;
double world_y_min;
double world_y_max;

// Waypoints
std::vector<point> waypoints;

// Path
int goal_visited = 1;
point goalPoint;
rrtTree* pathTree;
std::vector<point> path_RRT;
ros::ServiceClient gazebo_spawn;

// Robot
point robot_pose;
geometry_msgs::Twist cmd_vel;

// Point cloud data from kinect
pcl::PointCloud<pcl::PointXYZ> point_cloud;

#ifdef DEBUG_POINT_CLOUD
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr point_cloud_ptr (&point_cloud);
    pcl::visualization::CloudViewer cloud_viewer("CloudViewer");
#endif

// FSM state
int state;

// Function definition
bool isObstacle();
std::list<point>* getObstacleList();
bool isCleared();
std::list<point>* getClearedList();
inline bool isObstaclePoint(pcl::PointXYZ p);
inline bool isAvailablePoint(pcl::PointXYZ p);
bool isCollisionAndAddMargin();
bool isClearanceAndDeleteMargin();
void dynamic_mapping();
void set_waypoints();
void generate_path_RRT(point& from);
void generate_path_RRT(point& from, point& to);
void callback_state(gazebo_msgs::ModelStatesConstPtr msgs);
void callback_points(sensor_msgs::PointCloud2ConstPtr msgs);
inline void setcmdvel(double v, double w);
point transformFrameKinect2World(pcl::PointXYZ kinectPoint, point pointofRobot);
inline double getActualZ(pcl::PointXYZ kinectPoint);
cv::Mat addMargin(cv::Mat map, int margin);
void addNewMargin(cv::Mat& map, int margin, GridMapPoint& gp);
void deleteNewMargin(cv::Mat& map, int margin, GridMapPoint& gp);

#ifdef DEBUG_TEXT
    void printDebug(std::string str) {     
        std::cout<<str<<std::endl;
    }
#endif

#ifdef DEBUG_BALL
    void spawnBall(point p);

    std::string  ballLaunch = std::string("<robot name=\"simple_ball\">") +
        std::string("<link name=\"ball\">") +
        std::string("<inertial>") +
        std::string("<mass value=\"1.0\" />") +
        std::string("<origin xyz=\"0 0 0\" />") +
        std::string("<inertia  ixx=\"1.0\" ixy=\"0.0\"  ixz=\"0.0\"  iyy=\"1.0\"  iyz=\"0.0\"  izz=\"1.0\" />") +
        std::string("</inertial>") +
        std::string("<visual>") +
        std::string("<origin xyz=\"0 0 0\" rpy=\"0 0 0\" />") +
        std::string("<geometry>") +
        std::string("<sphere radius=\"0.09\"/>") +
        std::string("</geometry>") +
        std::string("</visual>") +
        std::string("<collision>") +
        std::string("<origin xyz=\"0 0 0\" rpy=\"0 0 0\" />") +
        std::string("<geometry>") +
        std::string("<sphere radius=\"0\"/>") +
        std::string("</geometry>") +
        std::string("</collision>") +
        std::string("</link>") +
        std::string("<gazebo reference=\"ball\">") +
        std::string("<mu1>10</mu1>") +
        std::string("<mu2>10</mu2>") +
        std::string("<material>Gazebo/Blue</material>") +
        std::string("<turnGravityOff>true</turnGravity Off>") +
        std::string("</gazebo>") +
        std::string("</robot>");
    int ballIdx = 0;
#endif

int main(int argc, char** argv) {
    ros::init(argc, argv, "rrt_main");

    // Initialize topics
    ros::NodeHandle n;
    ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/RosAria/cmd_vel", 1);
    ros::Subscriber gazebo_pose_sub = n.subscribe("/gazebo/model_states", 1, callback_state);
    ros::Subscriber gazebo_camera_sub = n.subscribe("/camera/depth/points", 1, callback_points);
    gazebo_spawn = n.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_urdf_model");
    ros::ServiceClient gazebo_set = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    printf("Initialize topics\n");

    // Load Map
    char* user = getlogin();
    map = cv::imread((std::string("/home/")+
                      std::string(user)+
                      std::string("/catkin_ws/src/project3/src/ground_truth_map.pgm")).c_str(), CV_LOAD_IMAGE_GRAYSCALE);
    map_y_range = map.cols;
    map_x_range = map.rows;
    map_origin_x = map_x_range/2.0 - 0.5;
    map_origin_y = map_y_range/2.0 - 0.5;
    world_x_min = -10.0;
    world_x_max = 10.0;
    world_y_min = -10.0;
    world_y_max = 10.0;
    res = 0.05;
    dynamic_map = addMargin(map, MARGIN);
    printf("Load map\n");

    // Set Way Points
    set_waypoints();
    printf("Set way points\n");

    // RRT
    generate_path_RRT(waypoints[0], waypoints[1]);
    printf("Generate RRT\n");

    // FSM
    state = INIT;
    bool running = true;
    purePursuit pure_pursuit;
    int look_ahead_idx;
    ros::Rate control_rate(10);

    while(running && ros::ok()){
        switch (state) {
        case INIT: {
            look_ahead_idx = 0;

            // Visualize path
#ifdef DEBUG_BALL
            if (VISUALIZE_PATH) {
                for(int i = 0; i < path_RRT.size(); i++){
                    spawnBall(path_RRT[i]);
                }
                printf("Spawn path\n");
            } else {
                printf("Skipped spawning path\n");
            }
#else
            printf("Skipped spawning path\n");
#endif

            //initialize robot position
            geometry_msgs::Pose model_pose;
            model_pose.position.x = waypoints[0].x;
            model_pose.position.y = waypoints[0].y;
            model_pose.position.z = 0.3;
            model_pose.orientation.x = 0.0;
            model_pose.orientation.y = 0.0;
            model_pose.orientation.z = 0.0;
            model_pose.orientation.w = 0.0;

            geometry_msgs::Twist model_twist;
            model_twist.linear.x = 0.0;
            model_twist.linear.y = 0.0;
            model_twist.linear.z = 0.0;
            model_twist.angular.x = 0.0;
            model_twist.angular.y = 0.0;
            model_twist.angular.z = 0.0;

            gazebo_msgs::ModelState modelstate;
            modelstate.model_name = "RosAria";
            modelstate.reference_frame = "world";
            modelstate.pose = model_pose;
            modelstate.twist = model_twist;

            gazebo_msgs::SetModelState setmodelstate;
            setmodelstate.request.model_state = modelstate;

            gazebo_set.call(setmodelstate);
            setcmdvel(0, 0);
            cmd_vel_pub.publish(cmd_vel);
            ros::spinOnce();
            ros::Rate(0.33).sleep();
            printf("Initialize ROBOT\n");

            state = RUNNING;
        } break;

        case RUNNING: {
            int iteration = 0; // # of while loop iteration
            purePursuit controller;
            std::vector<point>::iterator it = path_RRT.begin();
            int cc = 0;
            while(ros::ok()) {
                ++iteration;
                point nextPoint = *it;

                // Collision check
                if (iteration % COLLISION_CHECK_RATE == 0) {
                    // Scan quickly whether obstacle presents or not
                    bool obs = isObstacle();
                    bool clr = false; //isCleared();
                    if (obs || clr) {
                        // Stop and check carefully
                        setcmdvel(0, 0);
                        cmd_vel_pub.publish(cmd_vel);
                        ros::spinOnce();
                        ros::Duration(SLEEP_SETTLING_TIME).sleep();

                        bool isAddedMargin = false;
                        bool isDeletedMargin = false;
                        if (obs) {
                            isAddedMargin = isCollisionAndAddMargin();                            
                        }
                        if (clr) {
                            isDeletedMargin = isClearanceAndDeleteMargin();
                        }
                        if (isAddedMargin || isDeletedMargin) {
                            state = PATH_PLANNING;
                            break;
                        }
                    }
                }

                // End check
                if (robot_pose.distanceWith(nextPoint) < LOOK_AHEAD_DISTANCE) {
                    ++it;
                    if (it == path_RRT.end()) {
                        printf("Visited goal # %d\n", goal_visited);

                        ++goal_visited;
                        if (goal_visited == waypoints.size()) {
                            state = FINISH;
                            break;
                        }
                        goalPoint = waypoints[goal_visited];

                        setcmdvel(0, 0);
                        cmd_vel_pub.publish(cmd_vel);
                        ros::spinOnce();
                        ros::Duration(SLEEP_SETTLING_TIME).sleep();

                        state = PATH_PLANNING;
                        break;
                    }
                    continue;
                }

                control result = controller.get_control(robot_pose, nextPoint);
                setcmdvel(result.v, result.w);
                cmd_vel_pub.publish(cmd_vel);
                ros::spinOnce();
                control_rate.sleep();
            }
        } break;

        case PATH_PLANNING: {
            //TODO
            /*
             * do dynamic mapping from kinect data
             * pop up the opencv window
             * after drawing the dynamic map, transite the state to RUNNING state
             */
            printf("Path planning begin\n");

#ifdef DEBUG_IMAGE
            std::stringstream ss;
            ss << "wow" << imageNumberMain << ".jpg";
            ++imageNumberMain;
            cv::imwrite(ss.str().c_str(), dynamic_map);
#endif

            dynamic_mapping();
            printf("Path planning over\n");
            state = RUNNING;
        } break;

        case FINISH: {
            setcmdvel(0,0);
            cmd_vel_pub.publish(cmd_vel);
            running = false;

            ros::spinOnce();
            control_rate.sleep();
        } break;

        default: {
        } break;
        }
    }
    return 0;
}

void set_waypoints() {
    int order_size;
    switch (SCENARIO) {
    case 1: {

    } break;
    case 2: {

    } break;
    case 3: {
        point waypoint_candid[4];
        waypoint_candid[0].x = 5.0;
        waypoint_candid[0].y = -7.0;
        waypoint_candid[1].x = -3.0;
        waypoint_candid[1].y = -6.0;
        waypoint_candid[2].x = -8.0;
        waypoint_candid[2].y = 8.0;
        waypoint_candid[3].x = 8.0;
        waypoint_candid[3].y = 8.0;
        int order[] = {0,1,2,3};
        order_size = 4;
    } break;
    case 4: {
        point waypoint_candid[3];
        waypoint_candid[0].x = -6.0;
        waypoint_candid[0].y = 0.0;
        waypoint_candid[1].x = 3.0;
        waypoint_candid[1].y = -8.0;
        waypoint_candid[2].x = -8.0;
        waypoint_candid[2].y = 7.0;
        int order[] = {0,1,2};
        order_size = 3;
    } break;
    default: {
        point waypoint_candid[4];
        waypoint_candid[0].x = 5.0;
        waypoint_candid[0].y = -8.0;
        waypoint_candid[1].x = -6.0;
        waypoint_candid[1].y = -7.0;
        waypoint_candid[2].x = -8.0;
        waypoint_candid[2].y = 8.0;
        waypoint_candid[3].x = 3.0;
        waypoint_candid[3].y = 7.0;
        int order[] = {3,1,2,3};
        order_size = 4;
    } break;
    }
    
    for(int i = 0; i < order_size; i++){
        waypoints.push_back(waypoint_candid[order[i]]);
    }
}

// Generate path without changing goal
void generate_path_RRT(point& from) {
    int count;
    path_RRT.clear();
    pathTree = new rrtTree(from, goalPoint, map, dynamic_map, map_origin_x, map_origin_y, res);
    dynamic_map = pathTree->map.clone();
    while (true) {
        count = pathTree->generateRRTst(world_x_max, world_x_min, world_y_max, world_y_min, CONST_K, MAX_STEP);
        std::cout << "Path planning at (" << robot_pose.x << ", " << robot_pose.y << ")" << std::endl;
        GridMapPoint gp = GridMapPoint(robot_pose, res, map_origin_x, map_origin_y);
        std::cout << "Current position map value : " << (int)map.at<uchar>(gp.i,gp.j) << std::endl;
        if (count != -1) {
            std::cout << "Path planning successful" << std::endl;
            break;
        }
        std::cout << "Path planning failed" << std::endl;

    }
    std::vector<point> path;
    path = pathTree->backtracking();
    std::reverse(path.begin(), path.end());
    path_RRT.insert(path_RRT.end(), path.begin(), path.end());
    delete pathTree;
}

// Generate path with changing goal
void generate_path_RRT(point& from, point& to) {
    goalPoint = to;
    generate_path_RRT(from);
}

void callback_state(gazebo_msgs::ModelStatesConstPtr msgs) {
    for(int i; i < msgs->name.size(); i++){
        if(std::strcmp(msgs->name[i].c_str(),"RosAria") == 0){
            robot_pose.x = msgs->pose[i].position.x;
            robot_pose.y = msgs->pose[i].position.y;
            robot_pose.th = tf::getYaw(msgs->pose[i].orientation);
        }
    }
}

void callback_points(sensor_msgs::PointCloud2ConstPtr msgs) {
    pcl::fromROSMsg(*msgs,point_cloud);
}

<<<<<<< HEAD
/*
bool isObstacle() {
    pcl::PointCloud<pcl::PointXYZ> point_cloud_cpy = pcl::PointCloud<pcl::PointXYZ>(point_cloud);
    point robot_pose_cpy = robot_pose;
    point samplePoint;
    GridMapPoint gp = GridMapPoint(robot_pose_cpy, res, map_origin_x, map_origin_y);
    if (dynamic_map.at<uchar>(gp.i,gp.j) == 0) {
        return false;
    }
=======
inline bool isObstaclePoint(pcl::PointXYZ p) {
    return !(p.z != p.z)
        && getActualZ(p) > OBSTACLE_HEIGHT_MIN
        && getActualZ(p) < OBSTACLE_HEIGHT_MAX
        && p.z < OBSTACLE_DETERMINENT_DISTANCE
        && std::abs(p.x) < KINECT_VIEWAREA_X;
}
>>>>>>> 7771261454deaf6b929732303f5057c7ea0b58d6

inline bool isAvailablePoint(pcl::PointXYZ p) {
    return !(p.z != p.z)
        && (getActualZ(p) < CLEARANCE_HEIGHT_LOW
        || getActualZ(p) > CLEARANCE_HEIGHT_HIGH)
        && p.z < CLEARANCE_DETERMINENT_DISTANCE
        && std::abs(p.x) < KINECT_VIEWAREA_X;
}

std::list<point>* getClosestObstacleList() {
    std::list<point> *closestObstacleList = new std::list<point>();

    pcl::PointCloud<pcl::PointXYZ> point_cloud_cpy = pcl::PointCloud<pcl::PointXYZ>(point_cloud);
    point robot_pose_cpy = robot_pose;

    // Return if robot is on the margin
    point samplePoint;
    GridMapPoint gp = GridMapPoint(robot_pose_cpy, res, map_origin_x, map_origin_y);
    if (dynamic_map.at<uchar>(gp.i,gp.j) == 0) {
        return closestObstacleList;
    }

    // Get all obstacle points
    for (int w = 0; w < point_cloud_cpy.width; w += CAMERA_SAMPLING_RATE_W) {
        point farthestPoint;
        farthestPoint.th = -1;
        pcl::PointXYZ checkpoint;
        for (int h = point_cloud_cpy.height - 1; h > HORIZON - CHECK_Z_ABOVE_HORIZON; h -= CAMERA_SAMPLING_RATE_H) {
            checkpoint = point_cloud_cpy.at(w, h);
            if (isObstaclePoint(checkpoint)) {
                int obstaclePoints = 0;
                pcl::PointXYZ tempPoint;
                for (int delta_h = -OBSTACLE_CONFIRM_H; delta_h <= OBSTACLE_CONFIRM_H; ++delta_h) {
                    for (int delta_w = -OBSTACLE_CONFIRM_W; delta_w <= OBSTACLE_CONFIRM_W; ++delta_w) {
                        if (w + delta_w < 0 || w + delta_w >= point_cloud_cpy.width
                            || h + delta_h < 0 || h + delta_h >= point_cloud_cpy.height) {
                            continue;
                        }
                        tempPoint = point_cloud_cpy.at(w + delta_w, h + delta_h);
                        if (isObstaclePoint(tempPoint))
                            ++obstaclePoints;
                    }
                }
                if (obstaclePoints >= OBSTACLE_CONFIRM_THRESHOLD) {
                    samplePoint = transformFrameKinect2World(checkpoint, robot_pose_cpy);
                    gp = GridMapPoint(samplePoint, res, map_origin_x, map_origin_y);
                    if (gp.i >= 600 || gp.j >= 600 || gp.i < 200 || gp.j < 200)
                        continue;
<<<<<<< HEAD
                    if (dynamic_map.at<uchar>(gp.i, gp.j) > 0) {
                        obstacleList->push_back(samplePoint);
                    }
                }
            }
        }
    }
    return obstacleList;
}
*/
 /*
bool isCleared() {
    pcl::PointCloud<pcl::PointXYZ> point_cloud_cpy = pcl::PointCloud<pcl::PointXYZ>(point_cloud);
    point robot_pose_cpy = robot_pose;
    point samplePoint;
    GridMapPoint gp = GridMapPoint(robot_pose_cpy, res, map_origin_x, map_origin_y);
    if (dynamic_map.at<uchar>(gp.i,gp.j) == 0) {
        return false;
    }

    for (int h = point_cloud_cpy.height - 1; h > HORIZON - CHECK_Z_ABOVE_HORIZON; h -= CAMERA_SAMPLING_RATE_H) {
        for (int w = 0; w < point_cloud_cpy.width; w += CAMERA_SAMPLING_RATE_W) {
            pcl::PointXYZ checkpoint = point_cloud_cpy.at(w, h);
            if (isAvailablePoint(checkpoint)) {
                int clearedPoints = 0;
                pcl::PointXYZ tempPoint;
                for (int delta_h = -OBSTACLE_CONFIRM_H; delta_h <= OBSTACLE_CONFIRM_H; ++delta_h) {
                    for (int delta_w = -OBSTACLE_CONFIRM_W; delta_w <= OBSTACLE_CONFIRM_W; ++delta_w) {
                        if (w + delta_w < 0 || w + delta_w >= point_cloud_cpy.width
                            || h + delta_h < 0 || h + delta_h >= point_cloud_cpy.height) {
                            continue;
                        }
                        tempPoint = point_cloud_cpy.at(w + delta_w, h + delta_h);
                        if (isAvailablePoint(tempPoint))
                            ++clearedPoints;
                    }
                }
                if (clearedPoints >= CLEARANCE_CONFIRM_THRESHOLD) {
                    samplePoint = transformFrameKinect2World(checkpoint, robot_pose_cpy);
                    gp = GridMapPoint(samplePoint, res, map_origin_x, map_origin_y);
                    if (gp.i >= 600 || gp.j >= 600 || gp.i < 200 || gp.j < 200)
                        continue;
                    if (dynamic_map.at<uchar>(gp.i, gp.j) == 0)
                        return true;
                }
            }
        }
    }
    return false;
}

std::list<point>* getClearedList() {
    std::list<point> *clearedList = new std::list<point>();
    pcl::PointCloud<pcl::PointXYZ> point_cloud_cpy = pcl::PointCloud<pcl::PointXYZ>(point_cloud);
    point robot_pose_cpy = robot_pose;

    point samplePoint;
    GridMapPoint gp = GridMapPoint(robot_pose_cpy, res, map_origin_x, map_origin_y);
    if (dynamic_map.at<uchar>(gp.i,gp.j) == 0) {
        return clearedList;
    }

    for (int h = point_cloud_cpy.height - 1; h > HORIZON - CHECK_Z_ABOVE_HORIZON; h -= CAMERA_SAMPLING_RATE_H) {
        for (int w = 0; w < point_cloud_cpy.width; w += CAMERA_SAMPLING_RATE_W) {
            pcl::PointXYZ checkpoint = point_cloud_cpy.at(w, h);
            if (isAvailablePoint(checkpoint)) {
                int clearedPoints = 0;
                pcl::PointXYZ tempPoint;
                for (int delta_h = -CLEARANCE_CONFIRM_H; delta_h <= CLEARANCE_CONFIRM_H; ++delta_h) {
                    for (int delta_w = -CLEARANCE_CONFIRM_W; delta_w <= CLEARANCE_CONFIRM_W; ++delta_w) {
                        if (w + delta_w < 0 || w + delta_w >= point_cloud_cpy.width
                            || h + delta_h < 0 || h + delta_h >= point_cloud_cpy.height) {
                            continue;
                        }
                        tempPoint = point_cloud_cpy.at(w + delta_w, h + delta_h);
                        if (isAvailablePoint(tempPoint))
                            ++clearedPoints;
                    }
                }
                if (clearedPoints >= CLEARANCE_CONFIRM_THRESHOLD) {
                    samplePoint = transformFrameKinect2World(checkpoint, robot_pose_cpy);
                    gp = GridMapPoint(samplePoint, res, map_origin_x, map_origin_y);
                    if (gp.i >= 600 || gp.j >= 600 || gp.i < 200 || gp.j < 200)
                        continue;
                    if (dynamic_map.at<uchar>(gp.i, gp.j) == 0) {
                        clearedList->push_back(samplePoint);
                    }
=======
                    if (samplePoint.distanceWith(robot_pose_cpy) < farthestPoint.distanceWith(robot_pose_cpy))
                        farthestPoint = samplePoint;
>>>>>>> 7771261454deaf6b929732303f5057c7ea0b58d6
                }
            }
        }
        if (farthestPoint.th = -1)
            farthestPoint = transformFrameKinect2World(checkpoint, robot_pose_cpy);
        closestObstacleList.push_back(farthestPoint);
    }
<<<<<<< HEAD
    return clearedList;
}
*/

std::list<point>* getClosestObstacleList() {
    std::list<point> *closestObstacleList = new std::list<point>();

    pcl::PointCloud<pcl::PointXYZ> point_cloud_cpy = pcl::PointCloud<pcl::PointXYZ>(point_cloud);
    point robot_pose_cpy = robot_pose;

    // Return if robot is on the margin
    point samplePoint;
    GridMapPoint gp = GridMapPoint(robot_pose_cpy, res, map_origin_x, map_origin_y);
    if (dynamic_map.at<uchar>(gp.i,gp.j) == 0) {
        return closestObstacleList;
    }

    // Get all obstacle points
    for (int w = 0; w < point_cloud_cpy.width; w += CAMERA_SAMPLING_RATE_W) {
        point farthestPoint;
        farthestPoint.th = -1;
        pcl::PointXYZ checkpoint;
        for (int h = point_cloud_cpy.height - 1; h > HORIZON - CHECK_Z_ABOVE_HORIZON; h -= CAMERA_SAMPLING_RATE_H) {
            checkpoint = point_cloud_cpy.at(w, h);
            if (isObstaclePoint(checkpoint)) {
                int obstaclePoints = 0;
                pcl::PointXYZ tempPoint;
                for (int delta_h = -OBSTACLE_CONFIRM_H; delta_h <= OBSTACLE_CONFIRM_H; ++delta_h) {
                    for (int delta_w = -OBSTACLE_CONFIRM_W; delta_w <= OBSTACLE_CONFIRM_W; ++delta_w) {
                        if (w + delta_w < 0 || w + delta_w >= point_cloud_cpy.width
                            || h + delta_h < 0 || h + delta_h >= point_cloud_cpy.height) {
                            continue;
                        }
                        tempPoint = point_cloud_cpy.at(w + delta_w, h + delta_h);
                        if (isObstaclePoint(tempPoint))
                            ++obstaclePoints;
                    }
                }
                if (obstaclePoints >= OBSTACLE_CONFIRM_THRESHOLD) {
                    samplePoint = transformFrameKinect2World(checkpoint, robot_pose_cpy);
                    gp = GridMapPoint(samplePoint, res, map_origin_x, map_origin_y);
                    if (gp.i >= 600 || gp.j >= 600 || gp.i < 200 || gp.j < 200)
                        continue;
                    if (samplePoint.distanceWith(robot_pose_cpy) < farthestPoint.distanceWith(robot_pose_cpy))
                        farthestPoint = samplePoint;
                }
            }
        }
        if (farthestPoint.th = -1)
            farthestPoint = transformFrameKinect2World(checkpoint, robot_pose_cpy);
        closestObstacleList.push_back(farthestPoint);
    }

    return closestObstacleList;
}

bool updateMap() {
    bool result;
    std::list<point> *obstacleList = getObstacleList();

    if (result = !obstacleList->empty()) {
        std::list<point>::iterator it;
        GridMapPoint gp;
        for (it = obstacleList->begin(); it != obstacleList->end(); ++it) {
            gp = GridMapPoint(*it, res, map_origin_x, map_origin_y);
            if (gp.i >= 600 || gp.j >= 600 || gp.i < 200 || gp.j < 200)
                continue;
            addNewMargin(dynamic_map, NEW_OBSTACLE_MARGIN, gp);
        }
    }

    delete obstacleList;
    return result;
}
=======
>>>>>>> 7771261454deaf6b929732303f5057c7ea0b58d6

    return closestObstacleList;
}

<<<<<<< HEAD
inline bool isAvailablePoint(pcl::PointXYZ p) {
    return !(p.z != p.z)
        && (getActualZ(p) < CLEARANCE_HEIGHT_LOW
        || getActualZ(p) > CLEARANCE_HEIGHT_HIGH)
        && p.z < CLEARANCE_DETERMINENT_DISTANCE
        && std::abs(p.x) < KINECT_VIEWAREA_X;
}

 /*
bool isCollisionAndAddMargin() {
=======
bool isCollision() {
>>>>>>> 7771261454deaf6b929732303f5057c7ea0b58d6
    bool result;
    std::list<point> *closestObstacleList = getClosestObstacleList();

    processMap(closestObstacleList, int tolerance, point robotPose, pcl::map map);
    if (result = !obstacleList->empty()) {
        std::list<point>::iterator it;
        GridMapPoint gp;
        for (it = obstacleList->begin(); it != obstacleList->end(); ++it) {
            gp = GridMapPoint(*it, res, map_origin_x, map_origin_y);
            if (gp.i >= 600 || gp.j >= 600 || gp.i < 200 || gp.j < 200)
                continue;
            addNewMargin(dynamic_map, NEW_OBSTACLE_MARGIN, gp);
        }
    }

    delete obstacleList;
    return result;
}

<<<<<<< HEAD
bool isClearanceAndDeleteMargin() {
    bool result;
    std::list<point> *clearedList = getClearedList();

    if (result = !clearedList->empty()) {
        std::list<point>::iterator it;
        GridMapPoint gp;
        for (it = clearedList->begin(); it != clearedList->end(); ++it) {
            gp = GridMapPoint(*it, res, map_origin_x, map_origin_y);
            if (gp.i >= 600 || gp.j >= 600 || gp.i < 200 || gp.j < 200)
                continue;
            deleteNewMargin(dynamic_map, NEW_CLEARANCE_MARGIN, gp);
        }
    }

    delete clearedList;
    return result;
}
*/

=======
>>>>>>> 7771261454deaf6b929732303f5057c7ea0b58d6
void dynamic_mapping() {
    generate_path_RRT(robot_pose);
}

point transformFrameKinect2World(pcl::PointXYZ kinectPoint, point poseOfRobot) {
    point worldPoint;
    worldPoint.th = 0;

    double robot_x = kinectPoint.z;
    double robot_y = -kinectPoint.x;

    double cosTheta = cos(poseOfRobot.th);
    double sinTheta = sin(poseOfRobot.th);
    worldPoint.x = robot_x * cosTheta - robot_y * sinTheta + poseOfRobot.x;
    worldPoint.y = robot_x * sinTheta + robot_y * cosTheta + poseOfRobot.y;

    return worldPoint;
}

inline double getActualZ(pcl::PointXYZ kinectPoint) {
    return -kinectPoint.y + GROUND_HEIGHT;
}

cv::Mat addMargin(cv::Mat map, int margin) {
    cv::Mat map_margin = map.clone();
    int xSize = map.cols;
    int ySize = map.rows;

    for (int i = 0; i < ySize; i++) {
        for (int j = 0; j < xSize; j++) {
            if (map.at<uchar>(i, j) < 125) {
                for (int k = i - margin; k <= i + margin; k++) {
                    for (int l = j - margin; l <= j + margin; l++) {
                        if (k >= 0 && l >= 0 && k < ySize && l < xSize) {
                            map_margin.at<uchar>(k, l) = 0;
                        }
                    }
                }
            }
        }
    }

    return map_margin;
}

void setcmdvel(double v, double w) {
    cmd_vel.linear.x = v;
    cmd_vel.linear.y = 0.0;
    cmd_vel.linear.z = 0.0;
    cmd_vel.angular.x = 0.0;
    cmd_vel.angular.y = 0.0;
    cmd_vel.angular.z = w;
}

#ifdef DEBUG_BALL
    void spawnBall(point p) {
        gazebo_msgs::SpawnModel model;
        model.request.model_xml = ballLaunch;

        std::ostringstream ball_name;
        ball_name << "Ball " << ballIdx;
        model.request.model_name = ball_name.str();
        model.request.reference_frame = "world";
        model.request.initial_pose.position.x = p.x;
        model.request.initial_pose.position.y = p.y;
        model.request.initial_pose.position.z = 0.7;
        model.request.initial_pose.orientation.w = 0.0;
        model.request.initial_pose.orientation.x = 0.0;
        model.request.initial_pose.orientation.y = 0.0;
        model.request.initial_pose.orientation.z = 0.0;

        gazebo_spawn.call(model);
        ballIdx++;

        ros::spinOnce();
    }
#endif

inline bool comparePoints(point& p1, point& p2) {
    return p1.x > p1.x;
}

point rotate(int x, int y, double theta) {
    double x_new = x*cos(theta) - y*sin(theta);
    double y_new = x*sin(theta) + y*cos(theta);
    point newPoint;
    newPoint.x = x_new;
    newPoint.y = y_new;
    return newPoint;
}

std::list<GridMapPoint>* getMargins(point point1, point point2, int marginX, int marignY) { // pointx < point2.y 라는 가정
    std::list<GridMapPoint>* newList = new std::list<GridMapPoint>();
    GridMapPoint gp1 = GridMapPoint(point1, res, map_origin_x, map_origin_y);
    GridMapPoint gp2 = GridMapPoint(point2, res, map_origin_x, map_origin_y);
    double grad = (gp2.j - gp1.j) / (gp2.i - gp1.i);
    double distance = sqrt(pow((gp2.i - gp1.i),2) + pow((gp2.j - gp1.j),2));
    double theta = atan(grad);
    for (int i = -marginX; i < distance + marginX ; i++) {
        for (int j = -marginY; j < marginY; j++) {
            newList->push_back(rotate(i, j, theta));
        }
    }
    return newList;
}

void processMap(std::list<point> points, int tolerance, point robotPose, pcl::map map) {
    std::list<point>::iterator it = points.begin();
    std::list<point>::iterator it2 = next(points,1);
    while (it2 != points.end()) {
        point point1 = *it;
        point point2 = *it2;
        GridMapPoint pairOfRobot = GridMapPoint(robotPose);
        GridMapPoint pairs1 = GridMapPoint(point1);
        GridMapPoint pairs2 = GridMapPoint(point2);
        bool isObstacleInReal1 = points1.z != 100;
        bool isObstacleInReal2 = points2.z != 100;
        double gradInMap1 = (pairs1.i - pairOfRobot.i)/ (pairs1.j - pairOfRobot.j);
        double gradInMap2 = (pairs2.i - pairOfRobot.i)/ (pairs2.j - pairOfRobot.j);
        if (!isObstacleInReal && !isObstacleInReal1) { // 두 점 모두 clear 일경우 
            for (int y = pairOfRobot.j; y < pairs1.j; y++) {
                for (int x = gradInMap1 * (y - pairOfRobot.j) + pairOfRobot.i; x < gradInMap2 * (y - pairOfRobot.j) + pairOfRobot.i; x++) {
                    map.at(x, y) = 0;
                }
            }
        } else if (isObstacleInReal1 && isObstacleInReal2) { // 두 점 모두 obstacle
            for (int y = pairOfRobot.j; y < (pairs1.j - tolerance); y++) {
                for (int x = gradInMap1 * (y - pairOfRobot.j) + pairOfRobot.i; x < gradInMap2 * (y - pairOfRobot.j) + pairOfRobot.i; x++) {
                    map.at(x, y) = 0;
                }
            }
            bool isObstacleInMap = map.at(pairs.i, pairs.j);
            if (!isObstacleInMap) { // map상의 obstacle 이 있는지 확인
                std::list<GridMapPoint>* marginsList = getMargins(point1, point2 marginX, marignY); //point 1,2 사이의 buffer margin을 모두 가져온다 
                for(std::list<GridMapPoint>::iterator it = marginsList->begin(); it != marginsList->end(); it++) {
                    map.at(it->i, it->j) = 255;
                }
                delete marginsList;
            }
        } else { // 한점은 clear 한점은 obstacle
            point pointOfObstacle = isObstacleInReal1 ? points1 : points2;
            int yOfObastacle = getPoseOfMap(pointOfObstacle).j;
            for (int y = pairOfRobot.j; y < (yOfObastacle - tolerance); y++) {
                for (int x = gradInMap1 * (y - pairOfRobot.j) + pairOfRobot.i; x < gradInMap2 * (y - pairOfRobot.j) + pairOfRobot.i; x++) {
                    map.at(x, y) = 0;
                }
            }
        }
        it++;
        it2++;
    }
}
