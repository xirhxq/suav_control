#include "MyDataFun.h"
#include "MyMathFun.h"
#include "Utils.h"

#define KP 0.2
#define X_KP KP
#define Y_KP KP
#define Z_KP KP
#define YAW_KP 1.0 

class XY_CMD {
private:
    void setPositionCmd(double x, double y, double z) {
        cmd.data[0] = 1;
        cmd.data[1] = 1;
        cmd.data[3] = x;
        cmd.data[4] = y;
        cmd.data[5] = z;
        cmd.data[13] = 11;
    }
    void setVelocityCmd(double x, double y, double z) {
        cmd.data[0] = 2;
        cmd.data[1] = 2;
        cmd.data[6] = x;
        cmd.data[7] = y;
        cmd.data[8] = z;
        cmd.data[13] = 11;
    }
    void setYawCmd(double yaw) {
        cmd.data[2] = 1;
        cmd.data[9] = yaw * RAD2DEG_COE;
        cmd.data[13] = 11;
    }
    void setYawRateCmd(double yawRate) {
        cmd.data[2] = 3;
        cmd.data[9] = yawRate * RAD2DEG_COE;
        cmd.data[13] = 11;
    }
public:
    // Yaw & YawRate: in DEG!!!
    XY_CMD() {
        cmd.data.resize(15, 0.0); 
    }
    auto getPositionYawCmd(double x, double y, double z, double yaw){
        cmd.data.resize(15, 0.0);
        setPositionCmd(x, y, z);
        setYawCmd(yaw);
        return cmd;
    }
    auto getVelocityYawCmd(double x, double y, double z, double yaw){
        cmd.data.resize(15, 0.0);
        setVelocityCmd(x, y, z);
        setYawCmd(yaw);
        return cmd;
    }
    auto getVelocityYawRateCmd(double x, double y, double z, double yawRate){
        cmd.data.resize(15, 0.0);
        setVelocityCmd(x, y, z);
        setYawRateCmd(yawRate);
        return cmd;
    }
    auto getSpinCmd(){
        cmd.data.resize(15, 0.0);
        cmd.data[13] = 22;
        cmd.data[14] = 9;
        return cmd;
    }
    auto getTakeoffCmd(){
        cmd.data.resize(15, 0.0);
        cmd.data[13] = 22;
        cmd.data[14] = 1;
        return cmd;
    }
    auto getLandCmd(){
        cmd.data.resize(15, 0.0);
        cmd.data[13] = 22;
        cmd.data[14] = 16;
        return cmd;
    }
    auto getLockCmd(){
        cmd.data.resize(15, 0.0);
        cmd.data[13] = 22;
        cmd.data[14] = 40;
        return cmd;
    }
private:
    std_msgs::Float32MultiArray cmd;
};

class FLIGHT_CONTROL {
private:
    ros::NodeHandle nh;
    ros::Publisher ctrlCmdPub;
    ros::Subscriber flightDataSub, locSub;
    std::string cmd;
    XY_CMD xyCmd;

public:
    double currentHorMode, currentVerMode, currentYawMode;

    // RPY
    geometry_msgs::Vector3 currentEulerRad;

    // RPY
    geometry_msgs::Vector3 currentEulerDeg;

    geometry_msgs::Vector3 currentPos;

    geometry_msgs::Vector3 currentLocPos;

    geometry_msgs::Vector3 currentVel;

    double currentBaroHeight;

    sensor_msgs::NavSatFix currentGPS;

    double currentThrottle;

    geometry_msgs::Vector3 currentAcc;

    //0: auto; 1: remote control; 2 stabilize; 7: user mode
    double currentUAVStatus;

    double yawOffset;

    Point positionOffset;

    bool EMERGENCY = false;


    FLIGHT_CONTROL(std::string uavName, ros::NodeHandle nh_): nh(nh_){
        flightDataSub = nh.subscribe(uavName + "/xy_fcu/flight_data", 10, &FLIGHT_CONTROL::flightDataCallback, this);
        ctrlCmdPub = nh.advertise<std_msgs::Float32MultiArray>(uavName + "/fl5_guidance/guidance_cmds", 10);
        locSub = nh.subscribe(uavName + "/uwb/filter/odom", 10, &FLIGHT_CONTROL::locCallback, this);
    }

    void flightDataCallback(const std_msgs::Float32MultiArray &msgs) {
        currentHorMode = msgs.data[0];
        currentVerMode = msgs.data[1];
        currentYawMode = msgs.data[2];

        currentGPS.longitude = msgs.data[3];
        currentGPS.latitude  = msgs.data[4];
        currentGPS.altitude = msgs.data[5];

        currentBaroHeight = msgs.data[6];

        //enu
        currentVel.x = msgs.data[7];
        currentVel.y = msgs.data[8];
        currentVel.z = msgs.data[9];

        currentPos.x = msgs.data[10];
        currentPos.y = msgs.data[11];
        currentPos.z = msgs.data[12];

        currentEulerRad.x = msgs.data[13];
        currentEulerRad.y = msgs.data[14];
        currentEulerRad.z = msgs.data[15];

        MyDataFun::setValue(currentEulerDeg, MyDataFun::scale(currentEulerRad, RAD2DEG_COE));

        currentThrottle = msgs.data[16];

        currentAcc.x = msgs.data[17];
        currentAcc.y = msgs.data[18];
        currentAcc.z = msgs.data[19];

        currentUAVStatus = msgs.data[23];
    }

    void locCallback(const nav_msgs::Odometry::ConstPtr& msg){
        MyDataFun::setValue(currentLocPos, msg->pose.pose.position);
    }

    Point compensatePositionOffset(Point _p){
        Point res;
        res.x = _p.x + positionOffset.x;
        res.y = _p.y + positionOffset.y;
        res.z = _p.z;
        return res;
    }

    Point compensateOffset(Point _p){
        Point res;
        res.x = _p.x * cos(yawOffset) - _p.y * sin(yawOffset) + positionOffset.x;
        res.y = _p.x * sin(yawOffset) + _p.y * cos(yawOffset) + positionOffset.y;
        res.z = _p.z;
        return res;
    }

    Point compensateYawOffset(Point _p, double _y){
        Point res;
        res.x = _p.x * cos(_y) - _p.y * sin(_y);
        res.y = _p.x * sin(_y) + _p.y * cos(_y);
        res.z = _p.z;
        return res;
    }

    void cmdCallback(const std_msgs::String::ConstPtr& msg){
        cmd = msg->data;
        if (cmd == "0" || cmd == "emg" || cmd == "EMERGENCY"){
            EMERGENCY = true;
        }
        if (cmd == "233" || cmd == "ok"){
            EMERGENCY = false;
        }
    }

    template<typename T>
    bool isNear(T a, double r){
        return MyDataFun::dis(a, currentPos) <= r;
    }

    template<typename T>
    bool isNear2d(T a, double r){
        return MyDataFun::dis2d(a, currentPos) <= r;
    }

    void uavHoldCtrl() {
        ctrlCmdPub.publish(xyCmd.getVeloCmd(0, 0, 0, 0));
    }

    void uavAdjustYaw(double _yaw) {
        ctrlCmdPub.publish(xyCmd.getVeloCmd(0, 0, _yaw, 0));
    }

    void uavLand() {
        ctrlCmdPub.publish(xyCmd.getLandCmd());
    }

    void uavVelocityYawCtrl(double _vx, double _vy, double _vz, double _yaw) {
        ctrlCmdPub.publish(xyCmd.getVeloCmd(_vx, _vy, _vz, _yaw));
    }

    void uavPositionYawCtrl(double _x, double _y, double _z, double _yaw) {
        ctrlCmdPub.publish(xyCmd.getPositionCmd(_x, _y, _z, _yaw));
    }

    template<typename T>
    void uavVelocityYawCtrl(T posDiff, double yaw){
        T vel;
        vel.x = posDiff.x * X_KP;
        vel.y = posDiff.y * Y_KP;
        vel.z = posDiff.z * Z_KP;
        geometry_msgs::Vector3 sat;
        sat.x = 0.1;
        sat.y = 0.1;
        sat.z = 0.2;
        MyDataFun::saturateVel(vel, sat);
        ROS_INFO("Velo cmd: %s", MyDataFun::outputStr(vel).c_str());
        uavVelocityYawCtrl(vel.x, vel.y, vel.z, yaw);
    }

    template<typename T>
    void uavControlToPointFacingIt(T ctrlCmd){
        double yaw = MyDataFun::angle2d(currentPos, ctrlCmd);
        yaw = MyMathFun::radRound(yaw);
        if (MyDataFun::dis2d(ctrlCmd, currentPos) <= 1) yaw = 0;
        uavVelocityYawCtrl(MyDataFun::minus(ctrlCmd, currentPos), yaw);
    }

    template<typename T>
    void uavControlToPointWithYaw(T ctrlCmd, double yaw){
        yaw = MyMathFun::radRound(yaw);
        uavVelocityYawCtrl(MyDataFun::minus(ctrlCmd, currentPos), yaw);
    }

    double getTimeNow(){
        return ros::Time::now().toSec();
    }

    bool enoughTimeAfter(double _t0, double _duration){
        return getTimeNow() -_t0 >= _duration;
    }

};
