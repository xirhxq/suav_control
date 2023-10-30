#include "libFlightControl.hpp"

#define KP 0.2
#define X_KP KP
#define Y_KP KP
#define Z_KP KP
#define YAW_KP 1.0 

class XY_CMD {
private:
    double xVelSat = 0.5;
    double yVelSat = 0.5;
    double zVelSat = 2;
    double yawRateDegSat = 20;
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
        cmd.data[6] = LimitValue(x, xVelSat);
        cmd.data[7] = LimitValue(y, yVelSat);
        cmd.data[8] = LimitValue(z, zVelSat);
        cmd.data[13] = 11;
    }
    void setYawCmd(double yawDeg) {
        cmd.data[2] = 1;
        cmd.data[9] = degreeRound0To360(yawDeg) * DEG2RAD_COE;
        cmd.data[13] = 11;
    }
    void setYawRateCmd(double yawRateDeg) {
        cmd.data[2] = 3;
        cmd.data[9] = LimitValue(yawRateDeg, yawRateDegSat);
        cmd.data[13] = 11;
    }
    auto returnCmd() {
        printf("CMD: ");
        for (auto a: cmd.data) {
            printf("%.2f ", a);
        }
        printf("\n");
        return cmd;
    }
public:
    XY_CMD() {
        cmd.data.resize(15, 0.0); 
    }
    auto getPositionYawCmd(double x, double y, double z, double yawDeg){
        cmd.data.resize(15, 0.0);
        setPositionCmd(x, y, z);
        setYawCmd(yawDeg);
        return returnCmd();
    }
    auto getVelocityYawCmd(double x, double y, double z, double yawDeg){
        cmd.data.resize(15, 0.0);
        setVelocityCmd(x, y, z);
        setYawCmd(yawDeg);
        return returnCmd();
    }
    auto getVelocityYawRateCmd(double x, double y, double z, double yawRateDeg){
        cmd.data.resize(15, 0.0);
        setVelocityCmd(x, y, z);
        setYawRateCmd(yawRateDeg);
        return returnCmd();
    }
    auto getSpinCmd(){
        cmd.data.resize(15, 0.0);
        cmd.data[13] = 22;
        cmd.data[14] = 9;
        return returnCmd();
    }
    auto getTakeoffCmd(){
        cmd.data.resize(15, 0.0);
        cmd.data[13] = 22;
        cmd.data[14] = 1;
        return returnCmd();
    }
    auto getLandCmd(){
        cmd.data.resize(15, 0.0);
        cmd.data[13] = 22;
        cmd.data[14] = 16;
        return returnCmd();
    }
    auto getLockCmd(){
        cmd.data.resize(15, 0.0);
        cmd.data[13] = 22;
        cmd.data[14] = 40;
        return returnCmd();
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

    geometry_msgs::Vector3 currentRPYRad, currentRPYDeg;

    geometry_msgs::Vector3 currentPosXY;

    geometry_msgs::Vector3 currentPos;

    geometry_msgs::Vector3 currentVelENU;

    double currentBaroHeight;

    sensor_msgs::NavSatFix currentGPS;

    double currentThrottle;

    geometry_msgs::Vector3 currentAcc;

    //0: auto; 1: remote control; 2 stabilize; 7: user mode
    double currentUAVStatus;

    double yawOffsetDeg;

    Point positionOffset;

    bool EMERGENCY = false;


    FLIGHT_CONTROL(std::string uavName, ros::NodeHandle nh_): nh(nh_){
        flightDataSub = nh.subscribe(uavName + "/xy_fcu/flight_data", 10, &FLIGHT_CONTROL::flightDataCallback, this);
        ctrlCmdPub = nh.advertise<std_msgs::Float32MultiArray>(uavName + "/xy_fcu/xy_cmd", 10);
        locSub = nh.subscribe(uavName + "/filter/odom", 10, &FLIGHT_CONTROL::locCallback, this);
    }

    void flightDataCallback(const std_msgs::Float32MultiArray &msgs) {
        currentHorMode = msgs.data[0];
        currentVerMode = msgs.data[1];
        currentYawMode = msgs.data[2];

        currentGPS.longitude = msgs.data[3];
        currentGPS.latitude  = msgs.data[4];
        currentGPS.altitude = msgs.data[5];

        currentBaroHeight = msgs.data[6];

        currentVelENU.x = msgs.data[7];
        currentVelENU.y = msgs.data[8];
        currentVelENU.z = msgs.data[9];

        currentPosXY.x = msgs.data[10];
        currentPosXY.y = msgs.data[11];
        currentPosXY.z = msgs.data[12];

        currentRPYRad.x = msgs.data[13];
        currentRPYRad.y = msgs.data[14];
        currentRPYRad.z = msgs.data[15];

        setValue(currentRPYDeg, scale(currentRPYRad, RAD2DEG_COE));

        currentThrottle = msgs.data[16];

        currentAcc.x = msgs.data[17];
        currentAcc.y = msgs.data[18];
        currentAcc.z = msgs.data[19];

        currentUAVStatus = msgs.data[23];
    }

    void locCallback(const nav_msgs::Odometry::ConstPtr& msg){
        setValue(currentPos, msg->pose.pose.position);
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
        res.x = _p.x * cos(yawOffsetDeg) - _p.y * sin(yawOffsetDeg) + positionOffset.x;
        res.y = _p.x * sin(yawOffsetDeg) + _p.y * cos(yawOffsetDeg) + positionOffset.y;
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
        return dis(a, currentPos) <= r;
    }

    template<typename T>
    bool isNear2d(T a, double r){
        return dis2d(a, currentPos) <= r;
    }

    void uavSpin() {
        ctrlCmdPub.publish(xyCmd.getSpinCmd());
    }

    void uavTakeoff() {
        ctrlCmdPub.publish(xyCmd.getTakeoffCmd());
    }

    void uavLand() {
        ctrlCmdPub.publish(xyCmd.getLandCmd());
    }

    void uavLock() {
        ctrlCmdPub.publish(xyCmd.getLockCmd());
    }

    void uavHoldCtrl() {
        ctrlCmdPub.publish(xyCmd.getVelocityYawCmd(0, 0, 0, 0));
    }

    void uavAdjustYaw(double _yawDeg) {
        ctrlCmdPub.publish(xyCmd.getVelocityYawCmd(0, 0, _yawDeg, 0));
    }

    void uavVelocityYawCtrl(double _vx, double _vy, double _vz, double _yawDeg) {
        ctrlCmdPub.publish(xyCmd.getVelocityYawCmd(_vx, _vy, _vz, _yawDeg));
    }

    void uavPositionYawCtrl(double _x, double _y, double _z, double _yawDeg) {
        ctrlCmdPub.publish(xyCmd.getPositionYawCmd(_x, _y, _z, _yawDeg));
    }

    template<typename T>
    void uavVelocityYawKPCtrl(T posDiff, double yawDeg){
        T vel;
        vel.x = posDiff.x * X_KP;
        vel.y = posDiff.y * Y_KP;
        vel.z = posDiff.z * Z_KP;
        uavVelocityYawCtrl(vel.x, vel.y, vel.z, yawDeg);
    }

    template<typename T>
    void uavControlToPointFacingIt(T ctrlCmd){
        double yawDeg = angle2d(currentPos, ctrlCmd) * RAD2DEG_COE;
        yawDeg = degreeRound0To360(yawDeg);
        if (dis2d(ctrlCmd, currentPos) <= 1) yawDeg = 0;
        uavVelocityYawKPCtrl(minus(ctrlCmd, currentPos), yawDeg);
    }

    template<typename T>
    void uavControlToPointWithYaw(T ctrlCmd, double yawDeg){
        yawDeg = degreeRound(yawDeg);
        uavVelocityYawKPCtrl(minus(ctrlCmd, currentPos), yawDeg);
    }

    double getTimeNow(){
        return ros::Time::now().toSec();
    }

    bool enoughTimeAfter(double _t0, double _duration){
        return getTimeNow() -_t0 >= _duration;
    }

};
