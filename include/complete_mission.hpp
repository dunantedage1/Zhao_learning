#ifndef COMPLETE_MISSION_HPP
#define COMPLETE_MISSION_HPP

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <cmath>
#include <algorithm>

// 定义常量 - 任务参数
#define TARGET_ALTITUDE 1.5      // 目标高度1.5米
#define HOVER_DURATION 10.0      // 悬停10秒
#define FORWARD_DISTANCE 7.0     // 向前飞7米
// 门检测参数
#define GAP_MIN_WIDTH 0.6
#define GAP_MAX_WIDTH 2.0
#define CORNER_THRESHOLD 0.3
#define WINDOW_SIZE 5
#define MAX_DETECTION_RANGE 8.0
#define DETECTION_ANGLE_RANGE 45.0  // 只检测正前方±45°范围内的门

// 全局变量声明
extern mavros_msgs::PositionTarget setpoint_raw;
extern mavros_msgs::State current_state;
extern nav_msgs::Odometry local_pos;
extern int mission_num;

// 激光雷达数据
extern sensor_msgs::LaserScan laser_data;
extern bool obstacle_detected;

// 避障相关变量
extern float R_outside, R_inside;
extern float p_R, p_r;
extern float distance_c, angle_c;
extern float distance_cx, distance_cy;
extern float vel_collision[2];
extern float vel_collision_max;
extern float p_xy;
extern float vel_track[2];
extern float vel_track_max;
extern float vel_sp_body[2];
extern float vel_sp_ENU[2];
extern float vel_sp_max;
extern std_msgs::Bool flag_collision_avoidance;


struct Quaternion {
    double w, x, y, z;
};

struct EulerAngles {
    double roll, pitch, yaw;
};

struct GapGate {
    float start_angle;
    float end_angle;
    float width;
    float depth;
    float center_angle;
    float center_distance;
    float confidence;
    bool detected;
};

struct DetectedGate {
    float center_x, center_y;
    float width;
    float confidence;
    bool detected;
};

struct ComprehensiveGate {
    enum GateType {
        NO_GATE,
        POST_GATE,
        GAP_GATE
    };
    
    GateType type;
    DetectedGate post_gate;
    GapGate gap_gate;
    float center_x, center_y;
    float width;
    float confidence;
};

extern Quaternion q_fcu;
extern EulerAngles euler_fcu;

// 回调函数声明
void state_cb(const mavros_msgs::State::ConstPtr &msg);
void local_pos_cb(const nav_msgs::Odometry::ConstPtr &msg);
void laser_cb(const sensor_msgs::LaserScan::ConstPtr &msg);

// 任务函数声明
bool mission_pos_cruise(float x, float y, float z, float yaw, float error_max);
bool mission_forward_with_avoidance(float distance, float max_error);

// 避障算法函数
void cal_min_distance(const sensor_msgs::LaserScan& scan);
void collision_avoidance(float target_x, float target_y);
float satfunc(float data, float Max);
void rotation_yaw(float yaw_angle, float input[2], float output[2]);
EulerAngles quaternion_to_euler(const Quaternion& q);

// 辅助函数
void printf_mission_status();
void printf_obstacle_params();

#endif