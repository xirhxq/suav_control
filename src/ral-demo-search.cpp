#include "Utils.h"
#include "MyDataFun.h"
#include "MyMathFun.h"
#include "FlightControl.hpp"
#include "DataLogger.hpp"
#define CAMERA_ANGLE 30

using namespace dji_osdk_ros;
using namespace std;

typedef enum { TAKEOFF, ASCEND, SEARCH, TRACK, HOLD, BACK, LAND, END } ControlState;
ControlState task_state;
vector<geometry_msgs::Vector3> search_tra;
size_t search_tra_cnt;
double hold_begin_time, ascend_begin_time;
double task_begin_time, task_time;
double track_begin_time, track_time;
std::vector<std::pair<double, Point>> tracking_tra;
size_t track_tra_cnt;
Point desired_point;


void toStepTakeoff(){
    task_state = TAKEOFF;
    task_begin_time = get_time_now();
}

void toStepAscend() {
    ascend_begin_time = get_time_now();
    task_state = ASCEND;
}

void toStepTrack() {
    track_begin_time = get_time_now();
    track_tra_cnt = 0;
    task_state = TRACK;
}

void toStepSearch(){
    search_tra_cnt = 0;
    task_state = SEARCH;
}

void toStepHold(){
    task_state = HOLD;
    hold_begin_time = get_time_now();
}

void toStepLand(){
    task_state = LAND;
}

void toStepEnd() {
    task_state = END;
}

void StepTakeoff() {
    ROS_INFO("###----StepTakeoff----###");
    double expected_height = 1.4;
    ROS_INFO("Expected height @ %.2lf", expected_height);
    M210_position_yaw_rate_ctrl(0, 0, expected_height, 0);
    if (MyMathFun::nearly_is(current_pos_raw.z, expected_height, 0.2)){
        // ROS_INFO("Arrive expected height @ %.2lf", expected_height);
        toStepTrack();
        // toStepAscend();
        // toStepHold();
    }
}

void StepAscend(){
    ROS_INFO("###----StepAscend----###");
    double ascend_time = 3.0;
    ROS_INFO("Ascending by 0.1m/s: %.2lf", get_time_now() - ascend_begin_time);
    M210_hold_ctrl(0.1);
    if (enough_time_after(ascend_begin_time, ascend_time)){
        toStepHold();
    }
}

void StepTrack() {
    ROS_INFO("###----StepTrack----###");
    track_time = get_time_now() - track_begin_time;
    int sz = tracking_tra.size();
    while (track_tra_cnt < sz && tracking_tra[track_tra_cnt].first < track_time) {
        track_tra_cnt++;
    }
    if (track_tra_cnt == sz || track_time >= tracking_tra[sz - 1].first) {
        toStepHold();
        return;
    }

    Point pre_point, nxt_point;
    double pre_time, nxt_time;
    if (track_tra_cnt == 0) {
        pre_point = current_pos_raw;
        pre_time = 0.0;
    } else {
        pre_point = tracking_tra[track_tra_cnt - 1].second;
        pre_time = tracking_tra[track_tra_cnt - 1].first;
    }
    nxt_point = tracking_tra[track_tra_cnt].second;
    nxt_time = tracking_tra[track_tra_cnt].first;
    double ratio = (track_time - pre_time) / (nxt_time - pre_time);
    desired_point = MyDataFun::interpolate(pre_point, nxt_point, ratio);
    ROS_INFO("%s <------ %s ------> %s", 
        MyDataFun::output_str(pre_point).c_str(), 
        MyDataFun::output_str(desired_point).c_str(), 
        MyDataFun::output_str(nxt_point).c_str()
    );
    ROS_INFO("%.2lf <------ %.2lf ------> %.2lf\n", 
        pre_time, 
        track_time, 
        nxt_time
    );
    
    ROS_INFO("Go to %s(%ld th)", MyDataFun::output_str(desired_point).c_str(), track_tra_cnt);
    UAV_Control_to_Point_with_yaw(desired_point, yaw_offset);
}

void StepSearch() {
    ROS_INFO("###----StepSearch(Ground)----###");
    double tol = 0.3;
    MyDataFun::set_value(desired_point, search_tra[search_tra_cnt]);
    ROS_INFO("Go to %s(%ld th)", MyDataFun::output_str(desired_point).c_str(), search_tra_cnt);
    UAV_Control_to_Point_with_yaw(desired_point, yaw_offset);
    if (is_near(desired_point, tol)){
        search_tra_cnt++;
        if (search_tra_cnt == search_tra.size()){
            toStepHold();
        }
    }
}

void StepHold() {
    ROS_INFO("###----StepHold----###");
    double hold_time = 6.0;
    ROS_INFO("Hold %.2lf", get_time_now() - hold_begin_time);
    // M210_hold_ctrl(0.0);
    M210_adjust_yaw(yaw_offset);
    if (enough_time_after(hold_begin_time, hold_time)){
        toStepLand();
    }
}

void StepBack() {
    ROS_INFO("###----StepBack----###");
}

void StepLand() {
    ROS_INFO("###----StepLand----###");
    ROS_INFO("Landing...");
    takeoff_land(dji_osdk_ros::DroneTaskControl::Request::TASK_LAND);
    if (MyMathFun::nearly_is(current_pos_raw.z, 0.0, 0.2)) {
        toStepEnd();
    }
    // task_state = BACK;
}

void ControlStateMachine() {
    switch (task_state) {
        case TAKEOFF: {
            StepTakeoff();
            break;
        }
        case TRACK: {
            StepTrack();
            break;
        }
        case SEARCH: {
            StepSearch();
            break;
        }
        case HOLD: {
            StepHold();
            break;
        }
        case BACK: {
            StepBack();
            break;
        }
        case LAND: {
            StepLand();
            break;
        }
        default: {
            break;
        }
    }
}

bool load_search_tra(std::string name) {
    std::ifstream fin(std::string(ROOT_DIR) + "/config/online_tra_" + name + ".txt");
    if (!fin.is_open()) {
        ROS_ERROR("Cannot open %s/config/online_tra_%s.txt", std::string(ROOT_DIR).c_str(), name.c_str());
        return false;
    }
    double t, x, y, z;
    while (fin >> t >> x >> y >> z) {
        tracking_tra.push_back(std::make_pair(t, compensate_position_offset(MyDataFun::new_point(x, y, z))));
    }
    fin.close();
    ROS_INFO("Load search_tra.txt successfully");
    for (auto tra: tracking_tra) {
        ROS_INFO("Time: %.2lf Position: %s", tra.first, MyDataFun::output_str(tra.second).c_str());
    }
    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "mbzirc_demo_search", ros::init_options::AnonymousName);
    ros::NodeHandle nh;

    if (argc <= 2) {
        ROS_ERROR("Required more than 2 parameters, quitting...");
        return 0;
    }

    bool ON_GROUND = true;
    if (argc > 2) {
        auto on_ground = std::string(argv[2]);
        if (on_ground == "false" || on_ground == "takeoff") {
            ON_GROUND = false;
            ROS_WARN("WILL TAKE OFF!!!!");
        }
        else {
            ROS_WARN("ON GROUND TEST!!!");
        }
    }
    
	string uav_name = "none";
	if (argc > 1) {
		uav_name = std::string(argv[1]);
	}
    while (uav_name == "none"){
        ROS_ERROR("Invalid vehicle name: %s", uav_name.c_str());
    }
	ROS_INFO("Vehicle name: %s", uav_name.c_str());

    ros::Subscriber attitudeSub =
        nh.subscribe(uav_name + "/dji_osdk_ros/attitude", 10, &attitude_callback);
    ros::Subscriber gimbal_sub =
        nh.subscribe(uav_name + "/dji_osdk_ros/gimbal_angle", 10, &gimbal_callback);
    ros::Subscriber height_sub =
        nh.subscribe(uav_name + "/dji_osdk_ros/height_above_takeoff", 10, &height_callback);
    ros::Subscriber vo_pos_sub =
        nh.subscribe(uav_name + "/dji_osdk_ros/vo_position", 10, &vo_pos_callback);
    ros::Subscriber range_pos_sub = 
        nh.subscribe(uav_name + "/filter/odom", 10, &range_pos_callback);
	ros::Subscriber flightStatusSub =
			nh.subscribe(uav_name + "/dji_osdk_ros/flight_status", 10, &flight_status_callback);
	ros::Subscriber displayModeSub =
			nh.subscribe(uav_name + "/dji_osdk_ros/display_mode", 10, &display_mode_callback);
    ros::Subscriber cmd_sub = 
            nh.subscribe(uav_name + "/commander_cmd", 10, &cmd_callback);
    gimbal_angle_cmd_pub =
        nh.advertise<geometry_msgs::Vector3>(uav_name + "/gimbal_angle_cmd", 10);
    gimbal_speed_cmd_pub =
        nh.advertise<geometry_msgs::Vector3>(uav_name + "/gimbal_speed_cmd", 10);
    ros::Subscriber local_pos_sub = nh.subscribe(uav_name + "/dji_osdk_ros/local_position", 10, &local_pos_callback);

    ctrl_cmd_pub = nh.advertise<sensor_msgs::Joy>(
        uav_name + "/dji_osdk_ros/flight_control_setpoint_generic", 10);
    sdk_ctrl_authority_service =
        nh.serviceClient<dji_osdk_ros::SDKControlAuthority>(
            uav_name + "/dji_osdk_ros/sdk_control_authority");
    drone_task_service = nh.serviceClient<dji_osdk_ros::DroneTaskControl>(
        uav_name + "/dji_osdk_ros/drone_task_control");
    set_local_pos_reference = nh.serviceClient<dji_osdk_ros::SetLocalPosRef> (uav_name + "/dji_osdk_ros/set_local_pos_ref");

    ros::Rate rate(50);
    for (int i = 0; i < 100; i++) {
        ros::spinOnce();
        rate.sleep();
    }


    yaw_offset = current_euler_angle.z;
    ROS_INFO("Yaw offset: %.2lf", yaw_offset * RAD2DEG_COE);
    for (int i = 0; i < 100; i++) {
        ros::spinOnce();
        rate.sleep();
    }
    MyDataFun::set_value(position_offset, current_pos_raw);
    ROS_INFO("Position offset: %s", MyDataFun::output_str(position_offset).c_str());
   
    double x_dir = 1.0, y_dir = -1.0, z_height = 1.4; 
    search_tra.push_back(compensate_offset(MyDataFun::new_point(x_dir, 0.0, z_height)));
    search_tra.push_back(compensate_offset(MyDataFun::new_point(x_dir, y_dir, z_height)));
    search_tra.push_back(compensate_offset(MyDataFun::new_point(0, y_dir, z_height)));
    search_tra.push_back(compensate_offset(MyDataFun::new_point(0.0, 0.0, z_height)));

    ROS_INFO("Search Trajectory:");
    for (auto a: search_tra){
        ROS_INFO("%s", MyDataFun::output_str(a).c_str());
    }

    if (!load_search_tra(uav_name)) {
        return 0;
    }

    ROS_INFO("Use supersonic wave for height, now_height: %.2lf", current_pos_raw.z);
    string confirm_input;
    while (confirm_input != "yes"){
        ROS_INFO("Confirm: yes/no");
        cin >> confirm_input;
        if (confirm_input == "no"){
            return 0;
        }
    }
    
    DataLogger dl("search.csv");
    std::vector<std::pair<std::string, std::string> > vn = {
        {"taskTime", "double"},
        {"state", "enum"},
        {"pos", "Point"},
        {"eulerAngle", "Point"},
        {"trackTime", "double"},
        {"desiredPoint", "point"}
    };
    dl.initialize(vn);

    ROS_INFO("Waiting for command to take off...");
    sleep(3);
    // while(cmd != "ok"){
    //     ros::spinOnce();
    //     rate.sleep();
    // }
    if (!ON_GROUND) {
        obtain_control();
        monitoredTakeoff();
    }


    ROS_INFO("Start Control State Machine...");

    if (argc > 3) {
        // get the start state and set the task_state
        auto start_state = std::string(argv[3]);
        toStepTakeoff();
        if (start_state == "takeoff") {
            toStepTakeoff();
        }
        else if (start_state == "search") {
            toStepSearch();
        }
        else if (start_state == "land") {
            toStepLand();
        }
        else if (start_state == "track") {
            toStepTrack();
        }
        else if (start_state == "hold") {
            toStepHold();
        }
    }


    while (ros::ok()) {
        // std::cout << "\033c" << std::flush;
        task_time = get_time_now() - task_begin_time;
        ROS_INFO("-----------");
        ROS_INFO("Time: %lf", task_time);
        ROS_INFO("M210(State: %d) @ %s", task_state, MyDataFun::output_str(current_pos_raw).c_str());
        // ROS_INFO("Gimbal %s", MyDataFun::output_str(current_gimbal_angle).c_str());
        ROS_INFO("Attitude (R%.2lf, P%.2lf, Y%.2lf) / deg", current_euler_angle.x * RAD2DEG_COE,
                                                    current_euler_angle.y * RAD2DEG_COE,
                                                    current_euler_angle.z * RAD2DEG_COE);
        // if (!ON_GROUND) {
            if (EMERGENCY) {
                M210_hold_ctrl();
                printf("!!!!!!!!!!!!EMERGENCY!!!!!!!!!!!!\n");
            }
            else {
                ControlStateMachine();
                if (task_state == END) {
                    break;
                }
            }
        // }

        dl.log("taskTime", task_time);
        dl.log("state", task_state);
        dl.log("pos", current_pos_raw);
        dl.log("eulerAngle", current_euler_angle);
        dl.log("trackTime", track_time);
        dl.log("desiredPoint", desired_point);
        dl.newline();
        
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
