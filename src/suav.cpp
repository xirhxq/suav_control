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
    double tic, toc;

    DataLogger dl;
    ros::Rate rate;

    ros::Publisher uavReadyPub, uavStatePub;
    ros::Subscriber searchPointSub;
    int id = -1;
    bool searchOver;
    int searchState;

public:

    double hold_time_for_locateUSV;
    
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
        printf("Yaw offset: %.2lf\n", fc.yaw_offset * RAD2DEG_COE);
        for (int i = 0; i < 100; i++) {
            ros::spinOnce();
            rate.sleep();
        }
        MyDataFun::set_value(fc.position_offset, fc.current_pos_raw);
        printf("Position offset: %s\n", MyDataFun::output_str(fc.position_offset).c_str());

        printf("Use supersonic wave for height, now_height: %.2lf\n", fc.current_pos_raw.z);

        printf("Ignoring Search? %c\n", searchOver?'y':'n');
        string confirm_input;
        while (confirm_input != "yes"){
            printf("Confirm: yes/no\n");
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

        printf("Waiting for command to take off...\n");
        sleep(3);
        // while(id == -1 && ros::ok()){
        //      ros::spinOnce();
        //      rate.sleep();
        // }
        fc.set_local_position();
        if (!ON_GROUND) {
            fc.obtain_control();
            fc.monitoredTakeoff();
        }


        printf("Start Control State Machine...\n");

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
        tic = fc.get_time_now();
        task_state = TAKEOFF;
        task_begin_time = fc.get_time_now();
    }

    void toStepHold(){
        tic = fc.get_time_now();
        task_state = HOLD;
        hold_begin_time = fc.get_time_now();
    }

    void toStepLand(){
        tic = fc.get_time_now();
        task_state = LAND;
    }

    void toStepBack(){
        tic = fc.get_time_now();
        task_state = BACK;
    }

    void toStepEnd() {
        tic = fc.get_time_now();
        task_state = END;
    }

    void StepTakeoff() {
        printf("###----StepTakeoff----###\n");
        printf("Expected height @ %.2lf\n", expected_height);
        fc.M210_position_yaw_rate_ctrl(0, 0, expected_height, 0);
        if (toc - tic >= 15.0){
            // printf("Arrive expected height @ %.2lf\n", expected_height);
            toStepHold();
            uavReadyPub.publish(std_msgs::Empty());
        }
    }

    void StepHold() {
        printf("###----StepHold----###\n");
        auto expected_point = MyDataFun::new_point(0, 0, expected_height);
        printf("Hold %.2lf\n", fc.get_time_now() - hold_begin_time);
        printf("ExpectedPoint: %s\n", MyDataFun::output_str(expected_point).c_str());
        printf("Search over: %s\n", searchOver?"YES":"NO");
        fc.M210_adjust_yaw(fc.yaw_offset);
        // fc.UAV_Control_to_Point_with_yaw(expected_point, fc.yaw_offset);
        if (fc.enough_time_after(hold_begin_time, hold_time_for_locateUSV) && searchOver){
            toStepBack();
        }
    }

    void StepBack() {
        printf("###----StepBack----###\n");
        double hold_time = 5.0;
        auto expected_point = MyDataFun::new_point(0, 0, 2.0);
        printf("Back %.2lf\n", fc.get_time_now() - hold_begin_time);
        printf("ExpectedPoint: %s\n", MyDataFun::output_str(expected_point).c_str());
        printf("Search over: %s\n", searchOver?"YES":"NO");
        // fc.UAV_Control_to_Point_with_yaw(expected_point, fc.yaw_offset);
        fc.M210_position_yaw_rate_ctrl(0, 0, 2.0, 0);
        if (MyMathFun::nearly_is(fc.current_pos_raw.z, expected_point.z, 0.2)){
              toStepLand();
        }
    }

    void StepLand() {
        printf("###----StepLand----###\n");
        printf("Landing...\n");
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
            toc = fc.get_time_now();
            task_time = fc.get_time_now() - task_begin_time;
            printf("-----------\n");
            printf("Time: %lf\n", task_time);
            printf("M210(State: %d) @ %s\n", task_state, MyDataFun::output_str(fc.current_pos_raw).c_str());
            // printf("Gimbal %s\n", MyDataFun::output_str(current_gimbal_angle).c_str());
            printf("Attitude (R%.2lf, P%.2lf, Y%.2lf) / deg\n", fc.current_euler_angle.x * RAD2DEG_COE,
                                                        fc.current_euler_angle.y * RAD2DEG_COE,
                                                        fc.current_euler_angle.z * RAD2DEG_COE);
            printf("Search Over: %d\n", searchOver);                        
            printf("State time: %.2lf\n", toc - tic);
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

    std::string uav_name;
    bool ON_GROUND = false;
    bool ignoreSearch = true;
    std::string start_state;

    nh.getParam("uavName", uav_name);
    nh.getParam("ON_GROUND", ON_GROUND);
    nh.getParam("startState", start_state);
    nh.getParam("ignoreSearch", ignoreSearch);

    TASK t(uav_name, ON_GROUND, start_state, nh, ignoreSearch);
    nh.getParam("hold_time_for_locateUSV", t.hold_time_for_locateUSV);

    t.spin();
    return 0;
}
