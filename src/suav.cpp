#include "Utils.h"
#include "MyDataFun.h"
#include "MyMathFun.h"
#include "FlightControl.hpp"
#include "DataLogger.hpp"
#define CAMERA_ANGLE 30

using namespace dji_osdk_ros;
using namespace std;

class TASK {

private:

    FLIGHT_CONTROL fc;

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

    double expected_height = 7.5;

    DataLogger dl;
    ros::Rate rate;

    ros::Publisher uavReadyPub, uavStatePub;
    ros::Subscriber searchPointSub;
    int id = -1;
    bool searchOver;
    int searchState;

public:

    TASK(string name, bool ON_GROUND, string start_state, ros::NodeHandle nh_, bool ignoreSearch): 
        fc(name, nh_), dl("search.csv"), rate(50){
        
        uavReadyPub = nh_.advertise<std_msgs::Empty>(name + "/uavReady", 10);
        searchOver = ignoreSearch;
        
        uavStatePub = nh_.advertise<std_msgs::Int16>(name + "/uavState", 1);
        searchPointSub = nh_.subscribe(name + "/searchPoint", 1, &TASK::searchPointCallback, this);

        for (int i = 0; i < 100; i++) {
            ros::spinOnce();
            rate.sleep();
        }

        fc.yaw_offset = fc.current_euler_angle.z;
        ROS_INFO("Yaw offset: %.2lf", fc.yaw_offset * RAD2DEG_COE);
        for (int i = 0; i < 100; i++) {
            ros::spinOnce();
            rate.sleep();
        }
        MyDataFun::set_value(fc.position_offset, fc.current_pos_raw);
        ROS_INFO("Position offset: %s", MyDataFun::output_str(fc.position_offset).c_str());

        ROS_INFO("Use supersonic wave for height, now_height: %.2lf", fc.current_pos_raw.z);

        ROS_INFO("Ignoring Search? %c\n", searchOver?'y':'n');
        string confirm_input;
        while (confirm_input != "yes"){
            ROS_INFO("Confirm: yes/no");
            cin >> confirm_input;
            if (confirm_input == "no"){
                ROS_ERROR("Said no! Quit");
                assert(0);
            }
        }
    
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
         while(id == -1){
             ros::spinOnce();
             rate.sleep();
         }
        if (!ON_GROUND) {
            fc.obtain_control();
            fc.monitoredTakeoff();
        }


        ROS_INFO("Start Control State Machine...");

        toStepTakeoff();
        if (start_state == "takeoff") {
            toStepTakeoff();
        }
        else if (start_state == "land") {
            toStepLand();
        }
        else if (start_state == "hold") {
            toStepHold();
        }

    }

    void searchPointCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
        id = int(msg->data[0]);
        if (id == 6) {
            searchOver = true;
        }
    }

    void toStepTakeoff(){
        task_state = TAKEOFF;
        task_begin_time = fc.get_time_now();
    }

    void toStepHold(){
        task_state = HOLD;
        hold_begin_time = fc.get_time_now();
    }

    void toStepLand(){
        task_state = LAND;
    }

    void toStepBack(){
        task_state = BACK;
    }

    void toStepEnd() {
        task_state = END;
    }

    void StepTakeoff() {
        ROS_INFO("###----StepTakeoff----###");
        ROS_INFO("Expected height @ %.2lf", expected_height);
        fc.M210_position_yaw_rate_ctrl(0, 0, expected_height, 0);
        if (MyMathFun::nearly_is(fc.current_pos_raw.z, expected_height, 0.2)){
            // ROS_INFO("Arrive expected height @ %.2lf", expected_height);
            toStepHold();
            uavReadyPub.publish(std_msgs::Empty());
        }
    }

    void StepHold() {
        ROS_INFO("###----StepHold----###");
        double hold_time = 20.0;
        // auto expected_point = fc.compensate_yaw_offset(MyDataFun::new_point(10.0, 8.0, 2.0), fc.yaw_offset);
        auto expected_point = MyDataFun::new_point(0, 0, expected_height);
        ROS_INFO("Hold %.2lf", fc.get_time_now() - hold_begin_time);
        ROS_INFO("ExpectedPoint: %s", MyDataFun::output_str(expected_point).c_str());
        ROS_INFO("Search over: %s", searchOver?"YES":"NO");
        // fc.M210_adjust_yaw(fc.yaw_offset);
        fc.UAV_Control_to_Point_with_yaw(expected_point, fc.yaw_offset);
        if (fc.enough_time_after(hold_begin_time, hold_time) && searchOver){
            toStepBack();
        }
    }

    void StepBack() {
        ROS_INFO("###----StepBack----###");
        double hold_time = 5.0;
        auto expected_point = MyDataFun::new_point(0, 0, 2.0);
        ROS_INFO("Back %.2lf", fc.get_time_now() - hold_begin_time);
        ROS_INFO("ExpectedPoint: %s", MyDataFun::output_str(expected_point).c_str());
        ROS_INFO("Search over: %s", searchOver?"YES":"NO");
        fc.UAV_Control_to_Point_with_yaw(expected_point, fc.yaw_offset);
        if (MyMathFun::nearly_is(fc.current_pos_raw.z, expected_point.z, 0.2)){
              toStepLand();
        }
    }

    void StepLand() {
        ROS_INFO("###----StepLand----###");
        ROS_INFO("Landing...");
        fc.takeoff_land(dji_osdk_ros::DroneTaskControl::Request::TASK_LAND);
        if (MyMathFun::nearly_is(fc.current_pos_raw.z, 0.1, 0.2)) {
            toStepEnd();
        }
        // task_state = BACK;
    }

    void ControlStateMachine() {
        std_msgs::Int16 msg;
        msg.data = id * 100 + task_state * 10;
        if (task_state == HOLD) {
            msg.data += 1;
        }
        uavStatePub.publish(msg);
        switch (task_state) {
            case TAKEOFF: {
                StepTakeoff();
                break;
            }
            case HOLD: {
                StepHold();
                break;
            }
            case LAND: {
                StepLand();
                break;
            }
            case BACK: {
                StepBack();
                break;
            }
            default: {
                break;
            }
        }
    }

    void spin(){

        while (ros::ok()) {
            // std::cout << "\033c" << std::flush;
            task_time = fc.get_time_now() - task_begin_time;
            ROS_INFO("-----------");
            ROS_INFO("Time: %lf", task_time);
            ROS_INFO("M210(State: %d) @ %s", task_state, MyDataFun::output_str(fc.current_pos_raw).c_str());
            // ROS_INFO("Gimbal %s", MyDataFun::output_str(current_gimbal_angle).c_str());
            ROS_INFO("Attitude (R%.2lf, P%.2lf, Y%.2lf) / deg", fc.current_euler_angle.x * RAD2DEG_COE,
                                                        fc.current_euler_angle.y * RAD2DEG_COE,
                                                        fc.current_euler_angle.z * RAD2DEG_COE);
            ROS_INFO("Search Over: %d", searchOver);                        
            if (fc.EMERGENCY) {
                fc.M210_hold_ctrl();
                printf("!!!!!!!!!!!!EMERGENCY!!!!!!!!!!!!\n");
            }
            else {
                ControlStateMachine();
                if (task_state == END) {
                    break;
                }
            }

            dl.log("taskTime", task_time);
            dl.log("state", task_state);
            dl.log("pos", fc.current_pos_raw);
            dl.log("eulerAngle", fc.current_euler_angle);
            dl.log("trackTime", track_time);
            dl.log("desiredPoint", desired_point);
            dl.newline();
            
            ros::spinOnce();
            rate.sleep();
        }
    }

};


int main(int argc, char** argv) {
    ros::init(argc, argv, "suav", ros::init_options::AnonymousName);
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


    std::string start_state = "";
    if (argc > 3) {
        // get the start state and set the task_state
        start_state = std::string(argv[3]);
    }

    bool ignoreSearch = false;
    ROS_INFO("argc == %d, argv[4] == %s", argc, argv[4]);
    if (argc > 4 && std::string(argv[4]) == "ignoreSearch") {
        ROS_WARN("IGNORING SEARCH!!!");
        ignoreSearch = true;
    }


    TASK t(uav_name, ON_GROUND, start_state, nh, ignoreSearch);

    

    t.spin();
    return 0;
}
