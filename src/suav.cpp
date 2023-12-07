#include "FlightControlXY.hpp"
#include "DataLogger.hpp"

using namespace std;

class TASK {

private:

    FLIGHT_CONTROL fc;

    typedef enum {
        TAKEOFF = 0,
        ASCEND = 1,
        PREPARE = 2,
        HOLD = 3,
        BACK = 4,
        LAND = 5,
        END = 6
    } ControlState;
    ControlState taskState;

    // Point holdPoint1, holdPoint2, holdPoint3, holdPoint4, holdPoint5, holdPoint6, holdPoint7, holdPoint8, holdPoint9, holdPoint10, holdPoint11, holdPoint12, backPoint1;
    Point holdPoint1, holdPoint2, holdPoint3, holdPoint4, holdPoint5, backPoint1;

    vector<Point> ascendPoints;
    size_t ascendCnt;

    vector<Point> backPoints;
    size_t backCnt;

    vector<Point> holdPoints;
    vector<double> holdYawsDeg;
    vector<double> holdTimes;
    size_t holdCnt;

    double tic, toc;
    double taskBeginTime, taskTime;

    Point desiredPoint;
    double desiredYawDeg;

    DataLogger dl;
    ros::Rate rate;

    ros::Publisher uavStatePub;
    ros::Subscriber searchStateSub;
    bool searchOver;
    int searchState;

    bool onGround;

public:

    TASK(string name, bool ON_GROUND, string startState, ros::NodeHandle nh_, bool ignoreSearch):
        fc(name, nh_), dl("search.csv"), rate(50){

        searchOver = ignoreSearch;
        onGround = ON_GROUND;
        
        uavStatePub = nh_.advertise<std_msgs::Int8>(name + "/uavState", 1);
        searchStateSub = nh_.subscribe(name + "/pod/searchState", 1, &TASK::searchStateCallback, this);

        // for (int i = 0; i < 100; i++) {
        //     ros::spinOnce();
        //     rate.sleep();
        // }
        while(true)
        {
            printf("callback state: %d\n", fc.is_flight_state_updated);
            printf("Yaw offset / Deg: %.2lf\n", fc.yawOffsetDeg);
            if(fc.is_flight_state_updated == true)
                break;
            ros::spinOnce();
            rate.sleep();
        }


        fc.yawOffsetDeg = fc.currentRPYDeg.z;
        printf("Yaw offset / Deg: %.2lf\n", fc.yawOffsetDeg);

        ascendPoints = generateSmoothPath(
            positionOffsetPoint(0, 0, 5),
            positionOffsetPoint(100, 0, 40),
            5
        );

        printf("Ascend Points: \n");
        for (auto p: ascendPoints) {
            cout << outputStr(p) << endl;
        }
        printf("\n");

        backPoints = generateSmoothPath(
            positionOffsetPoint(100, 0, 40),
            positionOffsetPoint(0, 0, 5),
            5
        );

        printf("Back Points: \n");
        for (auto p: backPoints) {
            cout << outputStr(p) << endl;
        }
        printf("\n");

        holdYawsDeg = {
                // degreeRound0To360(fc.yawOffsetDeg + 0),
                // degreeRound0To360(fc.yawOffsetDeg + 0),
                // degreeRound0To360(fc.yawOffsetDeg + 0),
                // degreeRound0To360(fc.yawOffsetDeg + 0),
                // degreeRound0To360(fc.yawOffsetDeg + 0),
                // degreeRound0To360(fc.yawOffsetDeg + 0),
                // degreeRound0To360(fc.yawOffsetDeg + 0),

                degreeRound0To360(fc.yawOffsetDeg + 0),
                degreeRound0To360(fc.yawOffsetDeg + 0),
                degreeRound0To360(fc.yawOffsetDeg + 0),
                degreeRound0To360(fc.yawOffsetDeg + 0),
                degreeRound0To360(fc.yawOffsetDeg + 0)
//                degreeRound0To360(fc.yawOffsetDeg + 270),
//                degreeRound0To360(fc.yawOffsetDeg + 0)
        };

        for (int i = 0; i < 100; i++) {
            ros::spinOnce();
            rate.sleep();
        }

        fc.setPositionOffset();
        printf("Position offset ENU / m: %s\n", outputStr(fc.positionOffset).c_str());

        /*
        // forward
        holdPoint1 = fc.compensatePositionOffset(newPoint(0, 0, 5.0));
        holdPoint2 = fc.compensatePositionOffset(newPoint(0, -10.0, 5.0));
        holdPoint3 = fc.compensatePositionOffset(newPoint(0, -10.0, 10.0));
        holdPoint4 = fc.compensatePositionOffset(newPoint(0, -20.0, 10.0));
        holdPoint5 = fc.compensatePositionOffset(newPoint(0, -20.0, 15.0));

        // trajectory
        holdPoint6 = fc.compensatePositionOffset(newPoint(0, -100.0, 15.0));
        holdPoint7 = fc.compensatePositionOffset(newPoint(0, -200.0, 15.0));
        holdPoint8 = fc.compensatePositionOffset(newPoint(0, -340.0, 15.0));

        // backword
        holdPoint9 = fc.compensatePositionOffset(newPoint(0, -100, 15.0));
        holdPoint10 = fc.compensatePositionOffset(newPoint(0, -30.0, 15.0));
        holdPoint11 = fc.compensatePositionOffset(newPoint(0, 0.0, 15.0));
        holdPoint12 = fc.compensatePositionOffset(newPoint(0, 0, 5.0));
        */
        holdPoint1 = fc.compensatePositionOffset(newPoint(0, 0, 5.0));
        holdPoint2 = fc.compensatePositionOffset(newPoint(0, 0, 15.0));
        holdPoint3 = fc.compensatePositionOffset(newPoint(0, -50, 15.0));

        holdPoint4 = fc.compensatePositionOffset(newPoint(0, 0, 15.0));
        holdPoint5 = fc.compensatePositionOffset(newPoint(0, 0, 5.0));

        backPoint1 = fc.compensatePositionOffset(newPoint(0, 0, 0.5));
        
        // holdPoints = {holdPoint1, holdPoint2, holdPoint3, holdPoint4, holdPoint5, holdPoint6, holdPoint7, holdPoint8, holdPoint9, holdPoint10, holdPoint11, holdPoint12};
        // holdTimes = {5, 5, 5, 5, 10, 10, 10, 120, 10, 10, 5, 10};
        holdPoints = {holdPoint1, holdPoint2, holdPoint3, holdPoint4, holdPoint5};
        holdTimes = {5, 5, 20, 5, 10};
        
        printf("Hold Trajectory: \n");
        for (int i = 0; i < holdPoints.size(); i++) {
            printf("[%d]: %.2lf seconds @ %s with yaw %.2lf deg\n", i + 1, holdTimes[i], outputStr(holdPoints[i]).c_str(), holdYawsDeg[i]);
        }

        printf("Ignoring Search? %c\n", searchOver?'y':'n');
        printf("Start state input: %s\n", startState.c_str());

        string confirmInput;
        while (confirmInput != "yes"){
            printf("Confirm: yes/no\n");
            cin >> confirmInput;
            if (confirmInput == "no"){
                ROS_ERROR("Said no! Quit");
                assert(0);
            }
        }
    
        std::vector<std::pair<std::string, std::string> > vn = {
            {"taskTime", "double"},
            {"state", "enum"},
            {"stateNumber", "int"},
            {"stateTime", "double"},
            {"posENU", "Point"},
            {"rpyDeg", "Point"},
            {"desiredPoint", "point"},
            {"desiredYawDeg", "double"}
        };
        dl.initialize(vn);

        printf("Waiting for command to take off...\n");
        sleep(3);

        printf("Start Control State Machine...\n");

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

    int taskStateEncoder() const{
        return taskState * 10 + ((taskState == HOLD)? holdCnt: 0);
    }

    void toStepTakeoff(){
        taskState = TAKEOFF;
        taskBeginTime = fc.getTimeNow();
        tic = fc.getTimeNow();
    }

    void toStepAscend(){
        taskState = ASCEND;
        ascendCnt = 0;
        desiredPoint = ascendPoint[ascendCnt];
        desiredYawDeg = fc.yawOffsetDeg;
        tic = fc.getTimeNow();
    }

    void toStepPrepare(double restart=false){
        taskState = PREPARE;
        if (restart) {
            holdCnt = 0;
        }
        desiredPoint = holdPoints[holdCnt];
        desiredYawDeg = holdYawsDeg[holdCnt];
        tic = fc.getTimeNow();
    }

    void toStepHold(){
        taskState = HOLD;
        tic = fc.getTimeNow();
    }

    void toStepBack(){
        taskState = BACK;
        backCnt = 0;
        desiredPoint = backPoint[backCnt];
        desiredYawDeg = fc.yawOffsetDeg;
        tic = fc.getTimeNow();
    }

    void toStepLand(){
        taskState = LAND;
        tic = fc.getTimeNow();
    }

    void toStepEnd() {
        taskState = END;
        tic = fc.getTimeNow();
    }

    void StepTakeoff() {
        printf("----StepTakeoff----\n");
        if (!onGround) {
            if (toc - tic <= 3) fc.uavSpin();
            else fc.uavTakeoff();
        }
        if (fc.enoughTimeAfter(taskBeginTime, 5) && fc.currentPos.z >= 0.3){
            toStepAscend();
        }
    }

    void StepAscend() {
        printf("----StepAscend----\n");
        desiredPoint = ascendPoints[ascendCnt];
        fc.uavControlToPointWithYaw(desiredPoint, desiredYawDeg);
        if (fc.isNear(desiredPoint, 1.0)){
            ascendCnt++;
            if (ascendCnt == ascendPoints.size()){
                toStepPrepare(true);
            }
        }
    }

    void StepPrepare() {
        printf("----StepPrepare----\n");
        printf("Prepare to %ld/%ld point\n", holdCnt + 1, holdPoints.size());
        fc.uavControlToPointWithYaw(desiredPoint, desiredYawDeg);
        if (fc.isNear(desiredPoint, 1.0) && fc.yawNearDeg(desiredYawDeg, 5.0)){
            toStepHold();
        }
    }

    void StepHold() {
        printf("----StepHold----\n");
        printf("Hold to %ld/%ld point\n", holdCnt + 1, holdPoints.size());
        fc.uavControlToPointWithYaw(desiredPoint, desiredYawDeg);
        if (toc - tic >= holdTimes[holdCnt] && searchOver){
            holdCnt++;
            if (holdCnt >= holdPoints.size()) {
                toStepBack();
            }
            else {
                toStepPrepare(false);
            }
        }
    }

    void StepBack() {
        printf("----StepBack----\n");
        desiredPoint = backPoints[backCnt];
        fc.uavControlToPointWithYaw(desiredPoint, desiredYawDeg);
        if (fc.isNear(desiredPoint, 1.0)){
            backCnt++;
            if (backCnt == backPoints.size()){
                toStepLand();
            }
        }
    }

    void StepLand() {
        printf("----StepLand----\n");
        printf("Landing...\n");
        fc.uavLand();
        if (nearlyIs(fc.currentPos.z, 0.0, 0.2)) {
            toStepEnd();
        }
    }

    void ControlStateMachine() {
        toc = fc.getTimeNow();
        std_msgs::Int8 msg;
        msg.data = taskStateEncoder();
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
            case PREPARE: {
                StepPrepare();
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
            printf("-----------\n");
            printf("Task time: %.2lf, State time: %.2lf\n", taskTime, toc - tic);
            printf("suav (State: %d) @ %s\n", taskState, outputStr(fc.currentPos).c_str());
            printf("Desired Point: %s\n", outputStr(desiredPoint).c_str());
            printf("Desired Yaw: %.2lf Degf\n", desiredYawDeg);
            printf("Attitude (R%.2lf, P%.2lf, Y%.2lf) / deg\n", fc.currentRPYDeg.x, fc.currentRPYDeg.y, fc.currentRPYDeg.z);
            printf("Search Over: %d\n", searchOver);                        
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
            dl.log("stateNumber", taskStateEncoder());
            dl.log("stateTime", toc - tic);
            dl.log("posENU", fc.currentPos);
            dl.log("rpyDeg", fc.currentRPYDeg);
            dl.log("desiredPoint", desiredPoint);
            dl.log("desiredYawDeg", desiredYawDeg);
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
	printf("Vehicle name: %s\n", uavName.c_str());

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
