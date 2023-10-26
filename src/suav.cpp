#include "Utils.h"
#include "MyDataFun.h"
#include "MyMathFun.h"
#include "FlightControlXY.hpp"
#include "DataLogger.hpp"
#define CAMERA_ANGLE 30

using namespace std;

class TASK {

private:

    FLIGHT_CONTROL fc;

    typedef enum { TAKEOFF, ASCEND, SEARCH, TRACK, HOLD, BACK, LAND, END } ControlState;
    ControlState taskState;
    vector<geometry_msgs::Vector3> searchTra;
    size_t searchTraCnt;
    double holdBeginTime, ascendBeginTime;
    double taskBeginTime, taskTime;
    double trackBeginTime, trackTime;
    std::vector<std::pair<double, Point>> trackingTra;
    size_t trackTraCnt;
    Point desiredPoint;

    DataLogger dl;
    ros::Rate rate;

    ros::Publisher uavReadyPub, uavStatePub;
    ros::Subscriber searchStateSub;
    bool searchOver;
    int searchState;

public:

    TASK(string name, bool ON_GROUND, string startState, ros::NodeHandle nh_, bool ignoreSearch):
        fc(name, nh_), dl("search.csv"), rate(50){
        
        uavReadyPub = nh_.advertise<std_msgs::Empty>(name + "/uavReady", 10);
        searchOver = ignoreSearch;
        
        uavStatePub = nh_.advertise<std_msgs::Int8>(name + "/uavState", 1);
        searchStateSub = nh_.subscribe(name + "/pod/searchState", 1, &TASK::searchStateCallback, this);


        for (int i = 0; i < 100; i++) {
            ros::spinOnce();
            rate.sleep();
        }

        fc.yawOffsetDeg = fc.currentEulerDeg.z;
        ROS_INFO("Yaw offset: %.2lf", fc.yawOffsetDeg);
        for (int i = 0; i < 100; i++) {
            ros::spinOnce();
            rate.sleep();
        }
        MyDataFun::setValue(fc.positionOffset, fc.currentPos);
        ROS_INFO("Position offset: %s", MyDataFun::outputStr(fc.positionOffset).c_str());

        ROS_INFO("Use supersonic wave for height, now height: %.2lf", fc.currentPos.z);

        ROS_INFO("Ignoring Search? %c\n", searchOver?'y':'n');
        string confirmInput;
        while (confirmInput != "yes"){
            ROS_INFO("Confirm: yes/no");
            cin >> confirmInput;
            if (confirmInput == "no"){
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

        ROS_INFO("Start Control State Machine...");

        toStepTakeoff();
        if (startState == "takeoff") {
            toStepTakeoff();
        }
        else if (startState == "ascend") {
            toStepAscend();
        }
        else if (startState == "land") {
            toStepLand();
        }
        else if (startState == "hold") {
            toStepHold();
        }

    }

    void searchStateCallback(const std_msgs::Int8::ConstPtr& msg) {
        searchState = msg->data;
        if (searchState == 5) {
            searchOver = true;
        }
    }

    void toStepTakeoff(){
        taskState = TAKEOFF;
        taskBeginTime = fc.getTimeNow();
    }

    void toStepAscend(){
        taskState = ASCEND;
    }

    void toStepHold(){
        taskState = HOLD;
        holdBeginTime = fc.getTimeNow();
    }

    void toStepLand(){
        taskState = LAND;
    }

    void toStepBack(){
        taskState = BACK;
    }

    void toStepEnd() {
        taskState = END;
    }

    void StepTakeoff() {
        ROS_INFO("###----StepTakeoff----###");
        fc.uavTakeoff();
        if (fc.currentPos.z >= 0.5){
            toStepAscend();
        }
    }

    void StepAscend() {
        ROS_INFO("###----StepAscend----###");
        auto expectedPoint = MyDataFun::newPoint(0, 0, 1.0);
        ROS_INFO("Ascend To Point: %s", MyDataFun::outputStr(expectedPoint).c_str());
        fc.uavControlToPointWithYaw(expectedPoint, fc.yawOffsetDeg);
        if (MyMathFun::nearlyIs(fc.currentPos.z, expectedPoint.z, 0.2)){
            toStepHold();
        }
    }

    void StepHold() {
        ROS_INFO("###----StepHold----###");
        double holdTime = 6;
        // auto expectedPoint = fc.compensateYawOffset(MyDataFun::newPoint(10.0, 8.0, 2.0), fc.yawOffsetDeg);
        auto expectedPoint = MyDataFun::newPoint(0, 0, 1.0);
        ROS_INFO("Hold %.2lf", fc.getTimeNow() - holdBeginTime);
        ROS_INFO("ExpectedPoint: %s", MyDataFun::outputStr(expectedPoint).c_str());
        ROS_INFO("Search over: %s", searchOver?"YES":"NO");
        // fc.M210AdjustYaw(fc.yawOffsetDeg);
        fc.uavControlToPointWithYaw(expectedPoint, fc.yawOffsetDeg);
        if (fc.enoughTimeAfter(holdBeginTime, holdTime) && searchOver){
            toStepBack();
        }
    }

    void StepBack() {
        ROS_INFO("###----StepBack----###");
        auto expectedPoint = MyDataFun::newPoint(0, 0, 0.5);
        ROS_INFO("ExpectedPoint: %s", MyDataFun::outputStr(expectedPoint).c_str());
        ROS_INFO("Search over: %s", searchOver?"YES":"NO");
        fc.uavControlToPointWithYaw(expectedPoint, fc.yawOffsetDeg);
        if (MyMathFun::nearlyIs(fc.currentPos.z, expectedPoint.z, 0.2)){
            toStepLand();
        }
    }

    void StepLand() {
        ROS_INFO("###----StepLand----###");
        ROS_INFO("Landing...");
        fc.uavLand();
        if (MyMathFun::nearlyIs(fc.currentPos.z, 0.0, 0.2)) {
            toStepEnd();
        }
    }

    void ControlStateMachine() {
        std_msgs::Int8 msg;
        msg.data = taskState;
        uavStatePub.publish(msg);
        switch (taskState) {
            case TAKEOFF: {
                StepTakeoff();
                break;
            }
            case ASCEND: {
                StepAscend();
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
            taskTime = fc.getTimeNow() - taskBeginTime;
            ROS_INFO("-----------");
            ROS_INFO("Time: %.2lf", taskTime);
            ROS_INFO("suav (State: %d) @ %s", taskState, MyDataFun::outputStr(fc.currentPos).c_str());
            // ROS_INFO("Gimbal %s", MyDataFun::outputStr(currentGimbalAngle).c_str());
            ROS_INFO("Attitude (R%.2lf, P%.2lf, Y%.2lf) / deg", fc.currentEulerDeg.x, fc.currentEulerDeg.y, fc.currentEulerDeg.z);
            ROS_INFO("Search Over: %d", searchOver);                        
            if (fc.EMERGENCY) {
                fc.uavHoldCtrl();
                printf("!!!!!!!!!!!!EMERGENCY!!!!!!!!!!!!\n");
            }
            else {
                ControlStateMachine();
                if (taskState == END) {
                    break;
                }
            }

            dl.log("taskTime", taskTime);
            dl.log("state", taskState);
            dl.log("pos", fc.currentPos);
            dl.log("eulerAngle", fc.currentEulerDeg);
            dl.log("trackTime", trackTime);
            dl.log("desiredPoint", desiredPoint);
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
        auto onGround = std::string(argv[2]);
        if (onGround == "false" || onGround == "takeoff") {
            ON_GROUND = false;
            ROS_WARN("WILL TAKE OFF!!!!");
        }
        else {
            ROS_WARN("ON GROUND TEST!!!");
        }
    }
    
	string uavName = "none";
	if (argc > 1) {
		uavName = std::string(argv[1]);
	}
    while (uavName == "none"){
        ROS_ERROR("Invalid vehicle name: %s", uavName.c_str());
    }
	ROS_INFO("Vehicle name: %s", uavName.c_str());


    std::string startState = "";
    if (argc > 3) {
        // get the start state and set the taskState
        startState = std::string(argv[3]);
    }

    bool ignoreSearch = false;
    ROS_INFO("argc == %d, argv[4] == %s", argc, argv[4]);
    if (argc > 4 && std::string(argv[4]) == "ignoreSearch") {
        ROS_WARN("IGNORING SEARCH!!!");
        ignoreSearch = true;
    }


    TASK t(uavName, ON_GROUND, startState, nh, ignoreSearch);

    

    t.spin();
    return 0;
}
