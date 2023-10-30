#include "FlightControlXY.hpp"
#include "DataLogger.hpp"

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
    Point desiredPoint;

    DataLogger dl;
    ros::Rate rate;

    ros::Publisher uavStatePub;
    ros::Subscriber searchStateSub;
    bool searchOver;
    int searchState;

public:

    TASK(string name, bool ON_GROUND, string startState, ros::NodeHandle nh_, bool ignoreSearch):
        fc(name, nh_), dl("search.csv"), rate(50){

        searchOver = ignoreSearch;
        
        uavStatePub = nh_.advertise<std_msgs::Int8>(name + "/uavState", 1);
        searchStateSub = nh_.subscribe(name + "/pod/searchState", 1, &TASK::searchStateCallback, this);

        for (int i = 0; i < 100; i++) {
            ros::spinOnce();
            rate.sleep();
        }

        fc.yawOffsetDeg = fc.currentRPYDeg.z;
        printf("Yaw offset / Deg: %.2lf", fc.yawOffsetDeg);

        for (int i = 0; i < 100; i++) {
            ros::spinOnce();
            rate.sleep();
        }

        setValue(fc.positionOffset, fc.currentPos);
        printf("Position offset ENU / m: %s", outputStr(fc.positionOffset).c_str());

        printf("Ignoring Search? %c\n", searchOver?'y':'n');
        printf("Start state input: %s\n", startState.c_str());

        string confirmInput;
        while (confirmInput != "yes"){
            printf("Confirm: yes/no");
            cin >> confirmInput;
            if (confirmInput == "no"){
                ROS_ERROR("Said no! Quit");
                assert(0);
            }
        }
    
        std::vector<std::pair<std::string, std::string> > vn = {
            {"taskTime", "double"},
            {"state", "enum"},
            {"posENU", "Point"},
            {"rpyDeg", "Point"},
            {"desiredPoint", "point"}
        };
        dl.initialize(vn);

        printf("Waiting for command to take off...");
        sleep(3);

        printf("Start Control State Machine...");

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
        desiredPoint = newPoint(0, 3, 100.0);
    }

    void toStepHold(){
        taskState = HOLD;
        desiredPoint = newPoint(0, 3, 100.0);
        holdBeginTime = fc.getTimeNow();
    }

    void toStepLand(){
        taskState = LAND;
    }

    void toStepBack(){
        taskState = BACK;
        desiredPoint = newPoint(0, 3, 2.0);
    }

    void toStepEnd() {
        taskState = END;
    }

    void StepTakeoff() {
        printf("###----StepTakeoff----###");
        fc.uavTakeoff();
        if (fc.currentPos.z >= 0.3){
            toStepAscend();
        }
    }

    void StepAscend() {
        printf("###----StepAscend----###");
        fc.uavControlToPointWithYaw(desiredPoint, fc.yawOffsetDeg);
        if (nearlyIs(fc.currentPos.z, desiredPoint.z, 1)){
            toStepHold();
        }
    }

    void StepHold() {
        printf("###----StepHold----###");
        double holdTime = 6;
        printf("Hold %.2lf", fc.getTimeNow() - holdBeginTime);
        printf("ExpectedPoint: %s", outputStr(desiredPoint).c_str());
        printf("Search over: %s", searchOver?"YES":"NO");
        fc.uavControlToPointWithYaw(desiredPoint, fc.yawOffsetDeg);
        if (fc.enoughTimeAfter(holdBeginTime, holdTime) && searchOver){
            toStepBack();
        }
    }

    void StepBack() {
        printf("###----StepBack----###");
        printf("DesiredPoint: %s", outputStr(desiredPoint).c_str());
        printf("Search over: %s", searchOver?"YES":"NO");
        fc.uavControlToPointWithYaw(desiredPoint, fc.yawOffsetDeg);
        if (nearlyIs(fc.currentPos.z, desiredPoint.z, 1.0)){
            toStepLand();
        }
    }

    void StepLand() {
        printf("###----StepLand----###");
        printf("Landing...");
        fc.uavLand();
        if (nearlyIs(fc.currentPos.z, 0.0, 0.2)) {
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
            printf("-----------");
            printf("Time: %.2lf", taskTime);
            printf("suav (State: %d) @ %s", taskState, outputStr(fc.currentPos).c_str());
            printf("Desired Point: %s\n", outputStr(desiredPoint).c_str());
            printf("Attitude (R%.2lf, P%.2lf, Y%.2lf) / deg", fc.currentRPYDeg.x, fc.currentRPYDeg.y, fc.currentRPYDeg.z);
            printf("Search Over: %d", searchOver);                        
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
            dl.log("posENU", fc.currentPos);
            dl.log("rpyDeg", fc.currentRPYDeg);
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

	string uavName = std::string(argv[1]);
	printf("Vehicle name: %s", uavName.c_str());

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

    std::string startState = "";
    if (argc > 3) {
        startState = std::string(argv[3]);
    }

    bool ignoreSearch = false;
    if (argc > 4 && std::string(argv[4]) == "ignoreSearch") {
        ROS_WARN("IGNORING SEARCH!!!");
        ignoreSearch = true;
    }

    TASK t(uavName, ON_GROUND, startState, nh, ignoreSearch);

    t.spin();

    return 0;
}
