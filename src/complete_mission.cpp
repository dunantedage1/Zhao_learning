#include "complete_mission.hpp"
#include <chrono>

// 全局变量定义
mavros_msgs::PositionTarget setpoint_raw;
mavros_msgs::State current_state;
nav_msgs::Odometry local_pos;
int mission_num = 0;

// 激光雷达相关变量
sensor_msgs::LaserScan laser_data;
bool obstacle_detected = false;

bool once[7] = {0,0,0,0,0,0,0,};       // 有些调试内容反复输出太jb烦了

// 安全半径参数
float R_outside = 1.5;        // 外安全半径[米]，当障碍物距离小于此值时开始轻度避障
float R_inside = 0.6;          // 内安全半径[米]，当障碍物距离小于此值时开始强制避障

// 避障力系数
float p_R = 2.0;               // 外圈避障力比例系数，控制轻度避障的强度
float p_r = 3.0;               // 内圈避障力比例系数，控制强制避障的强度

// 障碍物检测相关
float distance_c = 0.0;        // 最近障碍物距离[米]
float angle_c = 0.0;           // 最近障碍物角度[度]，相对于机头方向
float distance_cx = 0.0;        // 最近障碍物在机体X轴方向的距离分量[米]
float distance_cy = 0.0;        // 最近障碍物在机体Y轴方向的距离分量[米]

// 避障速度分量
float vel_collision[2] = {0.0, 0.0};  // 避障产生的速度分量[m/s]，[0]=X方向，[1]=Y方向
float vel_collision_max = 1.0;  // 避障速度最大限幅[m/s]

// 追踪控制参数
float p_xy = 0.5;              // 位置追踪比例系数，控制向目标点移动的速度
float vel_track[2] = {0.0, 0.0};      // 目标追踪速度分量[m/s]，[0]=X方向，[1]=Y方向
float vel_track_max = 0.5;     // 追踪速度最大限幅[m/s]

// 合成速度
float vel_sp_body[2] = {0.0, 0.0};    // 机体坐标系下的合成速度[m/s]
float vel_sp_ENU[2] = {0.0, 0.0};     // ENU坐标系下的合成速度[m/s]
float vel_sp_max = 1.0;        // 总速度最大限幅[m/s]

bool gate_detected = false;     //穿门时候用的布尔量
bool gate_passed = false;       //检测是否已穿过这个门
ComprehensiveGate detected_gate;        //一个复杂门的结构体

// 避障状态标志
std_msgs::Bool flag_collision_avoidance;  // 避障模式标志，true=启用避障，false=正常追踪

// 位置和姿态
Quaternion q_fcu = {1.0, 0.0, 0.0, 0.0};        // 无人机当前姿态四元数
EulerAngles euler_fcu = {0.0, 0.0, 0.0};        // 无人机当前欧拉角[弧度],[0]=roll, [1]=pitch, [2]=yaw

// 初始化位置记录
float init_position_x_take_off = 0;
float init_position_y_take_off = 0;
float init_position_z_take_off = 0;
bool flag_init_position = false;

// 悬停计时器
std::chrono::steady_clock::time_point hover_start_time;
bool hover_timer_started = false;

/************************************************************************
 * AI写的数学工具，我看好像也没啥大用
 ************************************************************************/
 EulerAngles quaternion_to_euler(const Quaternion& q) {
    EulerAngles euler;
    
    // 四元数转欧拉角
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    euler.roll = std::atan2(sinr_cosp, cosr_cosp);
    
    double sinp = 2 * (q.w * q.y - q.z * q.x);
    if (std::abs(sinp) >= 1)
        euler.pitch = std::copysign(M_PI / 2, sinp);
    else
        euler.pitch = std::asin(sinp);
    
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    euler.yaw = std::atan2(siny_cosp, cosy_cosp);
    
    return euler;
}

/************************************************************************
 * 过滤无效数据，保留最近的障碍物数据
 ************************************************************************/
void cal_min_distance(const sensor_msgs::LaserScan& scan) {
    if (scan.ranges.empty()) return;
    
    int range_min = 0;    // 前向180度范围
    int range_max = scan.ranges.size() - 1;
    
    distance_c = scan.range_max;
    angle_c = 0;
    
    for (int i = range_min; i <= range_max; i++) {
        if (scan.ranges[i] < distance_c && !std::isinf(scan.ranges[i])) {
            distance_c = scan.ranges[i];
            angle_c = i;
        }
    }
    
    // 转换为直角坐标
    float angle_rad = (angle_c - 180) * M_PI / 180.0; // 转换为相对于机头的前向角度
    distance_cx = distance_c * std::cos(angle_rad);
    distance_cy = distance_c * std::sin(angle_rad);
}
/************************************************************************
 * 用于限制最大速度的函数
 ************************************************************************/
float speed_limit(float data, float Max) {
    if (std::abs(data) > Max) 
        return (data > 0) ? Max : -Max;
    else 
        return data;
}
/************************************************************************
 * 坐标旋转函数，用于转化到ENU坐标系
 ************************************************************************/
void rotation_yaw(float yaw_angle, float input[2], float output[2]) {
    output[0] = input[0] * std::cos(yaw_angle) - input[1] * std::sin(yaw_angle);
    output[1] = input[0] * std::sin(yaw_angle) + input[1] * std::cos(yaw_angle);
}
/************************************************************************
 * 基于人工势场的速度合成函数
 ************************************************************************/
void collision_avoidance(float target_x, float target_y) {
    // 计算当前位置
    float current_x = local_pos.pose.pose.position.x;
    float current_y = local_pos.pose.pose.position.y;
    
    // 2. 根据最小距离判断是否启用避障策略
    if (distance_c >= R_outside) {
        flag_collision_avoidance.data = false;
    } else {
        flag_collision_avoidance.data = true;
    }
    
    // 3. 计算追踪速度
    vel_track[0] = p_xy * (target_x - current_x);
    vel_track[1] = p_xy * (target_y - current_y);
    
    // 速度限幅
    for (int i = 0; i < 2; i++) {
        vel_track[i] = speed_limit(vel_track[i], vel_track_max);
    }
    
    vel_collision[0] = 0;
    vel_collision[1] = 0;
    
    // 4. 改进的避障策略 - 加入绕行能力
    if (flag_collision_avoidance.data == true) {
        float F_c = 0;
        float F_detour = 0;  // 绕行力系数
        
        if (distance_c > R_outside) {
            // 对速度不做限制
        } else if (distance_c > R_inside && distance_c <= R_outside) {
            // 小幅度抑制移动速度，并开始准备绕行
            F_c = p_R * (R_outside - distance_c);
            
            // 如果障碍物在正前方（角度绝对值小于45度），开始绕行准备
            if (std::abs(angle_c - 180) < 45) {
                F_detour = 1.5 * (R_outside - distance_c);
            }
        } else if (distance_c <= R_inside) {
            // 大幅度抑制移动速度，强制绕行
            F_c = p_R * (R_outside - R_inside) + p_r * (R_inside - distance_c);
            
            // 强制绕行，根据障碍物位置选择绕行方向
            F_detour = 2.5 * (R_inside - distance_c) + 0.5;
        }
        
        // 计算避障速度分量
        if (std::abs(distance_c) > 0.001) {
            // 主要避障力（远离障碍物）
            vel_collision[0] = -F_c * distance_cx / distance_c;
            vel_collision[1] = -F_c * distance_cy / distance_c;
            
            // 绕行力（侧向移动）- 解决三点一线问题
            if (F_detour > 0) {
                // 选择绕行方向：基于障碍物角度选择最优侧向
                // 角度转换为相对于机头前向的偏移（-180到180度，0度为正前方）
                float relative_angle = angle_c - 180;
                float detour_direction = (relative_angle >= 0) ? -1.0 : 1.0; // 障碍物在右侧则向左绕行
                
                // 计算侧向力：垂直于障碍物方向
                vel_collision[0] += -F_detour * distance_cy / distance_c * detour_direction;
                vel_collision[1] += F_detour * distance_cx / distance_c * detour_direction;
                
                ROS_WARN_THROTTLE(1, "绕行模式: 障碍物角度=%.1f°, 绕行方向=%.1f, 绕行力=%.2f", 
                                 relative_angle, detour_direction, F_detour);
            }
        }
        
        // 避障速度限幅
        for (int i = 0; i < 2; i++) {
            vel_collision[i] = speed_limit(vel_collision[i], vel_collision_max);
        }
    }
    
    // 5. 改进的速度合成策略 - 避免避障力与追踪力直接对抗
    float dot_product = vel_track[0] * vel_collision[0] + vel_track[1] * vel_collision[1];
    
    if (dot_product < -0.3 && flag_collision_avoidance.data) {
        // 避障力与追踪力方向相反（夹角大于90度），优先避障
        float avoidance_weight = 0.7;
        float track_weight = 0.3;
        
        vel_sp_body[0] = avoidance_weight * vel_collision[0] + track_weight * vel_track[0];
        vel_sp_body[1] = avoidance_weight * vel_collision[1] + track_weight * vel_track[1];
        
        ROS_WARN_THROTTLE(1, "力对抗检测: 优先避障, 避障权重=%.1f", avoidance_weight);
    } else {
        // 正常速度合成
        vel_sp_body[0] = vel_track[0] + vel_collision[0];
        vel_sp_body[1] = vel_track[1] + vel_collision[1];
    }
    
    // 总速度限幅
    for (int i = 0; i < 2; i++) {
        vel_sp_body[i] = speed_limit(vel_sp_body[i], vel_sp_max);
    }
    
    // 转换到ENU坐标系
    rotation_yaw(euler_fcu.yaw, vel_sp_body, vel_sp_ENU);
}
/************************************************************************
 * 回调函数，下面三个分别是当前状态，当前位置，当前雷达数据的回调函数
 ************************************************************************/
void state_cb(const mavros_msgs::State::ConstPtr &msg) {
    current_state = *msg;
}

void local_pos_cb(const nav_msgs::Odometry::ConstPtr &msg) {
    local_pos = *msg;
    
    // 记录起飞时的初始位置
    if (!flag_init_position && (local_pos.pose.pose.position.z > 0.05)) {
        init_position_x_take_off = local_pos.pose.pose.position.x;
        init_position_y_take_off = local_pos.pose.pose.position.y;
        init_position_z_take_off = local_pos.pose.pose.position.z;
        flag_init_position = true;
        ROS_INFO("初始位置记录完成: (%.2f, %.2f, %.2f)", 
                init_position_x_take_off, init_position_y_take_off, init_position_z_take_off);
    }
    
   // 更新姿态信息
    q_fcu.w = msg->pose.pose.orientation.w;
    q_fcu.x = msg->pose.pose.orientation.x;
    q_fcu.y = msg->pose.pose.orientation.y;
    q_fcu.z = msg->pose.pose.orientation.z;
    
    euler_fcu = quaternion_to_euler(q_fcu);
}

void laser_cb(const sensor_msgs::LaserScan::ConstPtr &msg) {
    laser_data = *msg;
    
    if (laser_data.ranges.empty()) {
        ROS_WARN_THROTTLE(5, "激光雷达数据为空");
        return;
    }
    
    // 处理激光数据并计算最近障碍物
    cal_min_distance(laser_data);
    
    // 设置障碍物检测标志
    obstacle_detected = (distance_c < R_outside);
    
    ROS_DEBUG_THROTTLE(2, "障碍物检测: 距离=%.2fm, 角度=%.1f°, 检测=%d", 
                      distance_c, angle_c, obstacle_detected);
}

/************************************************************************
 * 定点飞行到某个点，最后一个参数是最大误差
 ************************************************************************/
bool mission_pos_cruise(float x, float y, float z, float yaw, float error_max) {
    static bool mission_started = false;
    if (!mission_started) {
        mission_started = true;
    }
    
    // 设置目标位置（相对于起飞点）
    setpoint_raw.position.x = init_position_x_take_off + x;
    setpoint_raw.position.y = init_position_y_take_off + y;
    setpoint_raw.position.z = init_position_z_take_off + z;
    setpoint_raw.yaw = yaw;
    
    // 计算当前位置与目标位置的距离
    float dx = local_pos.pose.pose.position.x - setpoint_raw.position.x;
    float dy = local_pos.pose.pose.position.y - setpoint_raw.position.y;
    float dz = local_pos.pose.pose.position.z - setpoint_raw.position.z;
    
    float distance = std::sqrt(dx*dx + dy*dy + dz*dz);
    
    ROS_DEBUG_THROTTLE(2, "位置巡航: 距离目标=%.2fm", distance);
    
    // 如果距离小于误差范围，认为到达目标
    if (distance < error_max) {
        mission_started = false;
        return true;
    }
    
    return false;
}
/************************************************************************
 * 避障飞行到某个点，最后一个参数是最大误差
 ************************************************************************/
bool mission_forward_with_avoidance(float distance, float max_error) {
    static bool task_started = false;
    static float start_x = 0, start_y = 0;
    
    if (!task_started) {
        start_x = local_pos.pose.pose.position.x;
        start_y = local_pos.pose.pose.position.y;
        task_started = true;
        ROS_INFO("开始带避障的前向飞行%.1f米", distance);
    }
    
    // 计算目标位置（全局坐标）
    float target_x = start_x + distance;
    float target_y = start_y;
    
    // 如果X坐标已经大于等于6.6，目标点设为降落点
    float current_x = local_pos.pose.pose.position.x;
    if (current_x >= 6.6) {
        target_x = 8.0;
        target_y = 1.6;
        if(!once[0]){
        ROS_INFO("X坐标>=6.6m，目标点已设为(8,1.6,1.5)");
        once[0]=true;
        }
    }
    
    // 使用避障算法计算速度
    collision_avoidance(target_x, target_y);
    
    // 根据避障算法结果选择控制策略
    if (flag_collision_avoidance.data && current_x < 6.6) {
        // 避障模式：使用速度控制（仅在X<6.6时启用）
        setpoint_raw.type_mask = 0b000111000111; // 速度控制
        setpoint_raw.velocity.x = vel_sp_ENU[0];
        setpoint_raw.velocity.y = vel_sp_ENU[1];
        setpoint_raw.velocity.z = 0; // 保持高度
        setpoint_raw.yaw = 0;
        
        ROS_WARN_THROTTLE(1, "避障模式: vx=%.2f, vy=%.2f", 
                         vel_sp_ENU[0], vel_sp_ENU[1]);
    } else {
        // 正常模式：使用位置控制
        setpoint_raw.type_mask = 0b000111111000; // 位置控制
        setpoint_raw.position.x = target_x;
        setpoint_raw.position.y = target_y;
        setpoint_raw.position.z = TARGET_ALTITUDE;
        setpoint_raw.yaw = 0;
    }
    
    // 检查是否到达目标
    float dx = local_pos.pose.pose.position.x - target_x;
    float dy = local_pos.pose.pose.position.y - target_y;
    float current_distance = std::sqrt(dx*dx + dy*dy);
    
    ROS_DEBUG_THROTTLE(1, "前向飞行: 距离目标=%.2fm, 避障=%d", 
                      current_distance, flag_collision_avoidance.data);
    
    if (current_distance < max_error) {
        task_started = false;
        return true;
    }
    
    return false;
}
/************************************************************************
 * 获取正前方90°范围内的索引范围，穿门用的
 ************************************************************************/
void get_front_indices(const sensor_msgs::LaserScan& scan, int& start_idx, int& end_idx) {
    // 正前方是0°，范围是-45°到+45°
    float center_angle = 0.0f; // 正前方是0°
    float half_range_rad = 45.0f * M_PI / 180.0f;
    
    // 计算相对于angle_min的索引
    start_idx = (int)((center_angle - half_range_rad - scan.angle_min) / scan.angle_increment);
    end_idx = (int)((center_angle + half_range_rad - scan.angle_min) / scan.angle_increment);
    
    // 边界保护
    start_idx = std::max(0, start_idx);
    end_idx = std::min((int)scan.ranges.size() - 1, end_idx);
}
/************************************************************************
 * 基于拐角检测的门识别算法（只检测正前方90°范围内）
 ************************************************************************/
bool detect_gate_by_corners(const sensor_msgs::LaserScan& scan, float current_yaw, 
                           float drone_x, float drone_y, ComprehensiveGate& result) {
    result.type = ComprehensiveGate::NO_GATE;
    
    if (scan.ranges.empty()) return false;
    
    // 1. 获取正前方90°范围内的索引
    int start_idx, end_idx;
    get_front_indices(scan, start_idx, end_idx);
    
    if (start_idx >= end_idx) {
        ROS_DEBUG_THROTTLE(2, "无效的检测范围");
        return false;
    }
    
    // 2. 预处理数据（只处理检测范围内的数据）
    std::vector<float> processed_ranges(end_idx - start_idx + 1);
    for (int i = start_idx; i <= end_idx; i++) {
        int idx = i - start_idx;
        if (std::isinf(scan.ranges[i])) {
            processed_ranges[idx] = MAX_DETECTION_RANGE;
        } else {
            processed_ranges[idx] = scan.ranges[i];
        }
    }
    
    // 3. 寻找拐角点（局部极大值点）
    struct CornerPoint {
        int index;      // 在检测范围内的相对索引
        int global_index; // 在原始数据中的全局索引
        float distance;
        float confidence;
    };
    
    std::vector<CornerPoint> corners;
    
    // 使用滑动窗口检测拐角（只在检测范围内）
    for (int i = WINDOW_SIZE; i < processed_ranges.size() - WINDOW_SIZE; i++) {
        // 计算左侧平均值（靠近无人机的一侧）
        float left_avg = 0.0;
        for (int j = i - WINDOW_SIZE; j < i; j++) {
            left_avg += processed_ranges[j];
        }
        left_avg /= WINDOW_SIZE;
        
        // 计算右侧平均值（远离无人机的一侧）
        float right_avg = 0.0;
        for (int j = i + 1; j <= i + WINDOW_SIZE; j++) {
            right_avg += processed_ranges[j];
        }
        right_avg /= WINDOW_SIZE;
        
        // 当前点距离
        float current_dist = processed_ranges[i];
        
        // 检查是否是拐角：当前点距离明显大于两侧平均值
        if (current_dist > left_avg + CORNER_THRESHOLD && 
            current_dist > right_avg + CORNER_THRESHOLD) {
            
            // 计算拐角置信度
            float left_diff = current_dist - left_avg;
            float right_diff = current_dist - right_avg;
            float confidence = (left_diff + right_diff) / (2.0 * CORNER_THRESHOLD);
            
            int global_index = i + start_idx; // 转换为全局索引
            corners.push_back({i, global_index, current_dist, confidence});
        }
    }
    
    if (corners.size() < 2) {
        ROS_DEBUG_THROTTLE(2, "在正前方90°范围内找到 %zu 个拐角，需要至少2个", corners.size());
        return false;
    }
    
    // 4. 寻找成对的拐角（可能的门柱）
    struct GateCandidate {
        int left_corner_idx;      // 全局索引
        int right_corner_idx;     // 全局索引
        float width;
        float avg_distance;
        float confidence;
    };
    
    std::vector<GateCandidate> candidates;
    
    for (size_t i = 0; i < corners.size(); i++) {
        for (size_t j = i + 1; j < corners.size(); j++) {
            CornerPoint left_corner = corners[i];
            CornerPoint right_corner = corners[j];
            
            // 计算角度宽度
            float angle_width = (right_corner.global_index - left_corner.global_index) * scan.angle_increment;
            
            // 计算实际宽度
            float avg_dist = (left_corner.distance + right_corner.distance) / 2.0;
            float actual_width = 2.0 * avg_dist * tan(angle_width / 2.0);
            
            // 检查宽度是否合理
            if (actual_width < GAP_MIN_WIDTH || actual_width > GAP_MAX_WIDTH) continue;
            
            // 计算综合置信度
            float corner_conf = (left_corner.confidence + right_corner.confidence) / 2.0;
            float width_conf = 1.0 - fabs(actual_width - 1.0) / 1.0;

            
            float total_confidence = (corner_conf  + width_conf ) / 4.0;
            
            if (total_confidence > 0.4) {
                candidates.push_back({
                    left_corner.global_index, 
                    right_corner.global_index, 
                    actual_width, 
                    avg_dist, 
                    total_confidence
                });
            }
        }
    }
    
    if (candidates.empty()) {
        ROS_DEBUG_THROTTLE(2, "在正前方90°范围内找到 %zu 个拐角对，但没有合适的门候选", corners.size());
        return false;
    }
    
    // 5. 选择最佳候选（置信度最高且在前方）
    GateCandidate best_candidate = candidates[0];
    int center_index = start_idx + (end_idx - start_idx) / 2; // 检测范围的中心
    
    for (const auto& candidate : candidates) {
        int candidate_center = (candidate.left_corner_idx + candidate.right_corner_idx) / 2;
        int best_center = (best_candidate.left_corner_idx + best_candidate.right_corner_idx) / 2;
        
        // 优先选择正前方的门
        float candidate_offset = fabs(candidate_center - center_index);
        float best_offset = fabs(best_center - center_index);
        
        if (candidate.confidence > best_candidate.confidence * 0.8 && 
            candidate_offset < best_offset) {
            best_candidate = candidate;
        }
    }
    
    // 6. 填充结果
    result.type = ComprehensiveGate::GAP_GATE;
    result.gap_gate.start_angle = best_candidate.left_corner_idx * scan.angle_increment - scan.angle_max;
    result.gap_gate.end_angle = best_candidate.right_corner_idx * scan.angle_increment - scan.angle_max;
    result.gap_gate.center_angle = (best_candidate.left_corner_idx + best_candidate.right_corner_idx) / 2.0 * 
                                 scan.angle_increment - scan.angle_max;
    result.gap_gate.width = best_candidate.width;
    result.gap_gate.center_distance = best_candidate.avg_distance;
    result.gap_gate.confidence = best_candidate.confidence;
    result.gap_gate.detected = true;
    
    result.confidence = best_candidate.confidence;
    result.width = best_candidate.width;
    
    // 计算全局位置
    float global_angle = current_yaw + result.gap_gate.center_angle;
    result.center_x = drone_x + result.gap_gate.center_distance * cos(global_angle);
    result.center_y = drone_y + result.gap_gate.center_distance * sin(global_angle);
    
    ROS_INFO("正前方90°范围内检测到门: 距离=%.2fm, 宽度=%.2fm, 置信度=%.2f, 位置(%.2f, %.2f)", 
             result.gap_gate.center_distance, result.gap_gate.width, result.confidence,
             result.center_x, result.center_y);
    
    return true;
}

/************************************************************************
 * 状态显示函数，第一个用于打印任务状态，第二个用于打印避障参数
 ************************************************************************/
void printf_mission_status() {
    ROS_INFO_THROTTLE(5, "====== 任务状态 ======");
    ROS_INFO_THROTTLE(5, "任务阶段: %d", mission_num);
    ROS_INFO_THROTTLE(5, "当前位置: (%.2f, %.2f, %.2f)", 
                     local_pos.pose.pose.position.x,
                     local_pos.pose.pose.position.y,
                     local_pos.pose.pose.position.z);
    ROS_INFO_THROTTLE(5, "当前姿态: 翻滚=%.1f°, 俯仰=%.1f°, 偏航=%.1f°", 
                     euler_fcu.roll*180/M_PI, euler_fcu.pitch*180/M_PI, euler_fcu.yaw*180/M_PI);
}

void printf_obstacle_params() {
    if(mission_num <= 4){
    ROS_INFO_THROTTLE(5, "====== 避障状态 ======");
    ROS_INFO_THROTTLE(5, "最近障碍物: 距离=%.2fm, 角度=%.1f°", distance_c, angle_c);
    ROS_INFO_THROTTLE(5, "追踪速度: vx=%.2f, vy=%.2f", vel_track[0], vel_track[1]);
    ROS_INFO_THROTTLE(5, "避障速度: vx=%.2f, vy=%.2f", vel_collision[0], vel_collision[1]);
    ROS_INFO_THROTTLE(5, "总速度: vx=%.2f, vy=%.2f", vel_sp_ENU[0], vel_sp_ENU[1]);
    ROS_INFO_THROTTLE(5, "避障标志: %d", flag_collision_avoidance.data);
    }
}

/************************************************************************
 * 主函数
 ************************************************************************/
int main(int argc, char **argv) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "complete_mission");
    ros::NodeHandle nh;
    
    // 从参数服务器读取避障参数
    nh.param<float>("R_outside", R_outside, 1.5);
    nh.param<float>("R_inside", R_inside, 0.6);
    nh.param<float>("p_R", p_R, 2.0);
    nh.param<float>("p_r", p_r, 3.0);
    nh.param<float>("p_xy", p_xy, 0.5);
    nh.param<float>("vel_track_max", vel_track_max, 0.5);
    nh.param<float>("vel_collision_max", vel_collision_max, 1.0);
    nh.param<float>("vel_sp_max", vel_sp_max, 1.0);
    
    ROS_INFO("避障参数加载完成: R_outside=%.1f, R_inside=%.1f", R_outside, R_inside);
    
    // 订阅器
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>(
        "mavros/state", 10, state_cb);
    ros::Subscriber local_pos_sub = nh.subscribe<nav_msgs::Odometry>(
        "/mavros/local_position/odom", 10, local_pos_cb);
    ros::Subscriber laser_sub = nh.subscribe<sensor_msgs::LaserScan>(
        "/laser/scan", 10, laser_cb);
    
    // 发布器
    ros::Publisher setpoint_pub = nh.advertise<mavros_msgs::PositionTarget>(
        "/mavros/setpoint_raw/local", 10);
    
    // 服务客户端
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>(
        "mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>(
        "mavros/set_mode");
    
    ros::Rate rate(20);
    
    // 等待飞控连接
    ROS_INFO("等待飞控连接...");
    while (ros::ok() && !current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("飞控已连接");
    
    // 初始化控制指令
    setpoint_raw.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    setpoint_raw.type_mask = 0b000111111000; // 位置控制
    setpoint_raw.position.x = 0;
    setpoint_raw.position.y = 0;
    setpoint_raw.position.z = TARGET_ALTITUDE;
    setpoint_raw.yaw = 0;
    
    // 发送初始指令
    ROS_INFO("发送初始指令...");
    for (int i = 0; i < 100 && ros::ok(); ++i) {
        setpoint_pub.publish(setpoint_raw);
        ros::spinOnce();
        rate.sleep();
    }
    
    // 设置OFFBOARD模式并解锁
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    
    ros::Time last_request = ros::Time::now();
    bool offboard_set = false;
    bool armed = false;
    
    ROS_INFO("尝试设置OFFBOARD模式并解锁...");
    while (ros::ok() && (!offboard_set || !armed)) {
        if (!offboard_set && ros::Time::now() - last_request > ros::Duration(2.0)) {
            if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                offboard_set = true;
                ROS_INFO("OFFBOARD模式设置成功");
            }
            last_request = ros::Time::now();
        }
        
        if (!armed && ros::Time::now() - last_request > ros::Duration(2.0)) {
            if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
                armed = true;
                ROS_INFO("无人机解锁成功");
                mission_num = 1;
            }
            last_request = ros::Time::now();
        }
        
        setpoint_pub.publish(setpoint_raw);
        ros::spinOnce();
        rate.sleep();
    }
    
    // 主任务循环
    ROS_INFO("开始执行主任务...");
    
    // 门检测相关变量
    int gate_detection_attempts = 0;
    const int MAX_DETECTION_ATTEMPTS = 10;
    ros::Time last_detection_time = ros::Time::now();

    while (ros::ok()) {
        switch (mission_num) {
            case 1: // 起飞到目标高度
                if (mission_pos_cruise(0, 0, TARGET_ALTITUDE, 0, 0.1)) {
                    ROS_INFO("开始位置巡航到 (0,0,1.5)");
                    ROS_INFO("已达到目标高度1.5米，开始悬停10秒");
                    hover_start_time = std::chrono::steady_clock::now();
                    hover_timer_started = true;
                    mission_num = 2;
                }
                break;
                
            case 2: // 悬停
                if (hover_timer_started) {
                    auto now = std::chrono::steady_clock::now();
                    double hover_time = std::chrono::duration<double>(now - hover_start_time).count();
                    
                    if (hover_time >= HOVER_DURATION) {
                        ROS_INFO("悬停完成");
                        ROS_INFO("开始位置巡航到 (0,1.25,1.5)");
                            mission_num = 3;
                        
                    }
                    mission_pos_cruise(0, 0, TARGET_ALTITUDE, 0, 0.1);
                }
                break;

            case 3:
                 if(mission_pos_cruise(0,1.25,TARGET_ALTITUDE,0,0.1)){
                            ROS_INFO("已到达(0,1.25,1.5),开始带避障的向前飞行");
                            mission_num = 4;
                 }
                break;

            case 4: // 带避障的前向飞行
                if (mission_forward_with_avoidance(FORWARD_DISTANCE, 0.2)) {
                    ROS_INFO("向前飞行完成");
                    ROS_INFO("开始尝试穿门");
                    mission_num = 5;
                }
                break;

            case 5: // 使用拐角检测法检测门（只检测正前方90°范围内）
            {
                if (!gate_detected) {
                    // 每0.5秒检测一次
                    if (ros::Time::now() - last_detection_time > ros::Duration(0.5)) {
                        gate_detection_attempts++;
                        
                        if (detect_gate_by_corners(laser_data, euler_fcu.yaw,
                                local_pos.pose.pose.position.x,
                                local_pos.pose.pose.position.y,
                                detected_gate)) {
                            
                            gate_detected = true;
                            ROS_INFO("拐角检测成功！第%d次检测到门", gate_detection_attempts);
                            mission_num = 6;
                        } else {
                            ROS_INFO_THROTTLE(1, "第%d次拐角检测中（正前方90°范围）...", gate_detection_attempts);
                            
                            // 小范围移动帮助检测
                            if (gate_detection_attempts > 5) {
                                float search_y = 1.6 + 0.2 * sin(gate_detection_attempts * 0.5);
                                mission_pos_cruise(8.0, search_y, TARGET_ALTITUDE, 0, 0.1);
                            }
                            
                            // 如果多次检测失败，考虑其他策略
                            if (gate_detection_attempts > MAX_DETECTION_ATTEMPTS) {
                                ROS_WARN("拐角检测失败次数过多，尝试直接穿门");
                                // 使用预设的门位置
                                detected_gate.center_x = 11.0;
                                detected_gate.center_y = 1.6;
                                gate_detected = true;
                                mission_num = 6;
                            }
                        }
                        
                        last_detection_time = ros::Time::now();
                    }
                    
                    // 保持位置等待检测
                    mission_pos_cruise(8.0, 1.6, TARGET_ALTITUDE, 0, 0.2);
                }
                break;
            }
            
            case 6: // 飞向门中心
            {
                if (gate_detected) {
                    if(!once[1]){
                    ROS_INFO("飞向门中心: (%.2f, %.2f)", detected_gate.center_x, detected_gate.center_y);
                    once[1]=true;
                }
                    if (mission_pos_cruise(detected_gate.center_x, detected_gate.center_y, 
                                          TARGET_ALTITUDE, 0, 0.3)) {
                        ROS_INFO("已到达门中心，准备穿门");
                        mission_num = 7;
                    }
                } else {
                    // 如果没有检测到门，使用预设路径
                    ROS_WARN("使用预设路径穿门");
                    if (mission_pos_cruise(11.0, 1.6, TARGET_ALTITUDE, 0, 0.2)) {
                        mission_num = 7;
                    }
                }
                break;
            }
            
            case 7: // 穿过门
            {
                // 计算门后方1米的位置
                float target_x, target_y;
                
                if (gate_detected) {
                    // 使用检测到的门方向
                    target_x = detected_gate.center_x + 1.0 * cos(euler_fcu.yaw);
                    target_y = detected_gate.center_y + 1.0 * sin(euler_fcu.yaw);
                } else {
                    // 使用预设方向
                    target_x = 12.0;
                    target_y = 1.6;
                }
                if(!once[4]){
                ROS_INFO("穿过门到: (%.2f, %.2f)", target_x, target_y);
                once[4]=true;
                }
                
                if (mission_pos_cruise(target_x, target_y, TARGET_ALTITUDE, 0, 0.3)) {
                    ROS_INFO("成功穿过门！");
                    ROS_INFO("开始前往下一个门前");
                    mission_num = 8;
                }
                break;
            }

             case 8:
                 if(mission_pos_cruise(12.0,3.4,TARGET_ALTITUDE,0,0.1)){
                            ROS_INFO("开始尝试穿门");
                            gate_detected = false;
                            gate_detection_attempts = 0;
                            mission_num = 9;
                 }
                break;


            case 9: // 使用拐角检测法检测门（只检测正前方90°范围内）
            {
                if (!gate_detected) {
                    // 每0.5秒检测一次
                    if (ros::Time::now() - last_detection_time > ros::Duration(0.5)) {
                        gate_detection_attempts++;
                        
                        if (detect_gate_by_corners(laser_data, euler_fcu.yaw,
                                local_pos.pose.pose.position.x,
                                local_pos.pose.pose.position.y,
                                detected_gate)) {
                            
                            gate_detected = true;
                            ROS_INFO("拐角检测成功！第%d次检测到门", gate_detection_attempts);
                            mission_num = 10;
                        } else {
                            ROS_INFO_THROTTLE(1, "第%d次拐角检测中（正前方90°范围）...", gate_detection_attempts);
                            
                            // 小范围移动帮助检测
                            if (gate_detection_attempts > 5) {
                                float search_y = 3.4 + 0.2 * sin(gate_detection_attempts * 0.5);
                                mission_pos_cruise(12.0, search_y, TARGET_ALTITUDE, 0, 0.1);
                            }
                            
                            // 如果多次检测失败，考虑其他策略
                            if (gate_detection_attempts > MAX_DETECTION_ATTEMPTS) {
                                ROS_WARN("拐角检测失败次数过多，尝试直接穿门");
                                // 使用预设的门位置
                                detected_gate.center_x = 14.0;
                                detected_gate.center_y = 3.4;
                                gate_detected = true;
                                mission_num = 10;
                            }
                        }
                        
                        last_detection_time = ros::Time::now();
                    }
                    
                    // 保持位置等待检测
                    mission_pos_cruise(12.0, 3.4, TARGET_ALTITUDE, 0, 0.2);
                }
                break;
            }
            
            case 10: // 飞向门中心
            {
                if (gate_detected) {
                    if(!once[2]){
                    ROS_INFO("飞向门中心: (%.2f, %.2f)", detected_gate.center_x, detected_gate.center_y);
                    once[2]=true;
                }
                    
                    if (mission_pos_cruise(detected_gate.center_x, detected_gate.center_y, 
                                          TARGET_ALTITUDE, 0, 0.3)) {
                        ROS_INFO("已到达门中心，准备穿门");
                        mission_num = 11;
                    }
                } else {
                    // 如果没有检测到门，使用预设路径
                    ROS_WARN("使用预设路径穿门");
                    if (mission_pos_cruise(16.0, 3.4, TARGET_ALTITUDE, 0, 0.2)) {
                        mission_num = 11;
                    }
                }
                break;
            }
            
            case 11: // 穿过门
            {
                // 计算门后方1米的位置
                float target_x, target_y;
                
                if (gate_detected) {
                    // 使用检测到的门方向
                    target_x = detected_gate.center_x + 1.0 * cos(euler_fcu.yaw);
                    target_y = detected_gate.center_y + 1.0 * sin(euler_fcu.yaw);
                } else {
                    // 使用预设方向
                    target_x = 15.0;
                    target_y = 3.4;
                }
                
                if(!once[5]){
                ROS_INFO("穿过门到: (%.2f, %.2f)", target_x, target_y);
                once[5]=true;
                }
                
                if (mission_pos_cruise(target_x, target_y, TARGET_ALTITUDE, 0, 0.3)) {
                    ROS_INFO("成功穿过门！");
                    ROS_INFO("开始升高，准备第三次穿门");
                    mission_num = 12;
                }
                break;
            }

            case 12:
                 if(mission_pos_cruise(15.0,3.4,TARGET_ALTITUDE*2,0,0.1)){
                            ROS_INFO("已到达指定高度3m，向第三个门前进");
                            mission_num = 13;
                 }
                break;

            case 13:
                 if(mission_pos_cruise(9.0,-0.5,TARGET_ALTITUDE*2,0,0.1)){
                            ROS_INFO("准备下降到1.5m");
                            mission_num = 14;
                 }
                break;

            case 14:
                 if(mission_pos_cruise(9.0,-0.5,TARGET_ALTITUDE,0,0.1)){
                            ROS_INFO("准备第三次尝试穿门");
                            gate_detected = false;
                            gate_detection_attempts = 0;
                            mission_num = 15;
                 }
                break;

            case 15: // 使用拐角检测法检测门（只检测正前方90°范围内）
            {
                if (!gate_detected) {
                    // 每0.5秒检测一次
                    if (ros::Time::now() - last_detection_time > ros::Duration(0.5)) {
                        gate_detection_attempts++;
                        
                        if (detect_gate_by_corners(laser_data, euler_fcu.yaw,
                                local_pos.pose.pose.position.x,
                                local_pos.pose.pose.position.y,
                                detected_gate)) {
                            
                            gate_detected = true;
                            ROS_INFO("拐角检测成功！第%d次检测到门", gate_detection_attempts);
                            mission_num = 16;
                        } else {
                            ROS_INFO_THROTTLE(1, "第%d次拐角检测中（正前方90°范围）...", gate_detection_attempts);
                            
                            // 小范围移动帮助检测
                            if (gate_detection_attempts > 5) {
                                float search_y = -0.5 + 0.2 * sin(gate_detection_attempts * 0.5);
                                mission_pos_cruise(9.0, search_y, TARGET_ALTITUDE, 0, 0.2);
                            }
                            
                            // 如果多次检测失败，考虑其他策略
                            if (gate_detection_attempts > MAX_DETECTION_ATTEMPTS) {
                                ROS_WARN("拐角检测失败次数过多，尝试直接穿门");
                                // 使用预设的门位置
                                detected_gate.center_x = 12.0;
                                detected_gate.center_y = -0.5;
                                gate_detected = true;
                                mission_num = 16;
                            }
                        }
                        
                        last_detection_time = ros::Time::now();
                    }
                    
                    // 保持位置等待检测
                    mission_pos_cruise(9, -0.5, TARGET_ALTITUDE, 0, 0.2);
                }
                break;
            }
            
            case 16: // 飞向门中心
            {
                if (gate_detected) {
                    if(!once[3]){
                    ROS_INFO("飞向门中心: (%.2f, %.2f)", detected_gate.center_x, detected_gate.center_y);
                    once[3]=true;
                }
                    
                    if (mission_pos_cruise(detected_gate.center_x, detected_gate.center_y, 
                                          TARGET_ALTITUDE, 0, 0.3)) {
                        ROS_INFO("已到达门中心，准备穿门");
                        mission_num = 17;
                    }
                } else {
                    // 如果没有检测到门，使用预设路径
                    ROS_WARN("使用预设路径穿门");
                    if (mission_pos_cruise(12.0, -0.5, TARGET_ALTITUDE, 0, 0.3)) {
                        mission_num = 17;
                    }
                }
                break;
            }
            
            case 17: // 穿过门
            {
                // 计算门后方1米的位置
                float target_x, target_y;
                
                if (gate_detected) {
                    // 使用检测到的门方向
                    target_x = detected_gate.center_x + 1.0 * cos(euler_fcu.yaw);
                    target_y = detected_gate.center_y + 1.0 * sin(euler_fcu.yaw);
                } else {
                    // 使用预设方向
                    target_x = 13.0;
                    target_y = -0.5;
                }
                
                if(!once[6]){
                ROS_INFO("穿过门到: (%.2f, %.2f)", target_x, target_y);
                once[6]=true;
                }
                
                if (mission_pos_cruise(target_x, target_y, TARGET_ALTITUDE, 0, 0.3)) {
                    ROS_INFO("成功穿过门！");
                    ROS_INFO("前往指定地点降落");
                    mission_num = 18;
                }
                break;
            }

            case 18:
                 if(mission_pos_cruise(13.0,-0.5,TARGET_ALTITUDE*2,0,0.1)){
                            ROS_INFO("已升至安全高度，开始前进");
                            mission_num = 19;
                 }
                break;

            case 19:
                 if(mission_pos_cruise(35.0,1.0,TARGET_ALTITUDE*2,0,0.1)){
                            ROS_INFO("准备降落");
                            mission_num = 20;
                 }
                break;
                
            case 20: // 降落
                offb_set_mode.request.custom_mode = "AUTO.LAND";
                if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                    ROS_INFO("降落模式已启动");
                    mission_num = -1;
                }
                break;
                
            case -1: // 任务完成
                ROS_INFO_THROTTLE(5, "任务已完成，等待降落...");
                break;
        }
        
        // 发布控制指令
        setpoint_pub.publish(setpoint_raw);
        
        // 显示状态信息
        printf_mission_status();
        printf_obstacle_params();
        
        ros::spinOnce();
        rate.sleep();
    }
    
    ROS_INFO("程序结束");
    return 0;
}
