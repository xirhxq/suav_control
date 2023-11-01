#include "libFlightControl.hpp"

#define KP 0.2
#define X_KP KP
#define Y_KP KP
#define Z_KP KP
#define YAW_KP 1.0

#define UWB_POS
//#define GPS_POS

#if defined(UWB_POS) && defined(GPS_POS)
#error "UWB_POS and GPS_POS cannot be defined at the same time"
#elif !defined(UWB_POS) && !defined(GPS_POS)
#error "UWB_POS and GPS_POS must be defined at least one"
#endif

class XY_CMD {
private:
    double eVelSat = 0.5, nVelSat = 0.5, uVelSat = 2;
    double pDegSat = 5, rDegSat = 5;
    double pRadSat = pDegSat * DEG2RAD_COE, rRadSat = rDegSat * DEG2RAD_COE;
    double fAccSat = 0.2, lAccSat = 0.2, uAccSat = 0.2;
    double yawRateDegSat = 20;
    void setHorPosition(double ePos, double nPos) {
        cmd.data[0] = 1;
        cmd.data[3] = ePos;
        cmd.data[4] = nPos;
        cmd.data[13] = 11;
    }
    void setHorVelocity(double eVel, double nVel) {
        cmd.data[0] = 2;
        cmd.data[6] = LimitValue(eVel, eVelSat);
        cmd.data[7] = LimitValue(nVel, nVelSat);
        cmd.data[13] = 11;
    }
    void setHorAttitude(double pRad, double rRad) {
        cmd.data[0] = 3;
        cmd.data[10] = LimitValue(radRound(pRad), pRadSat);
        cmd.data[11] = LimitValue(radRound(rRad), rRadSat);
        cmd.data[13] = 11;
    }
    void setHorAcceleration(double fAcc, double lAcc) {
        cmd.data[0] = 4;
        cmd.data[6] = LimitValue(fAcc, fAccSat);
        cmd.data[7] = LimitValue(lAcc, lAccSat);
        cmd.data[13] = 11;
    }
    void setVerPosition(double uPos) {
        cmd.data[1] = 1;
        cmd.data[5] = uPos;
        cmd.data[13] = 11;
    }
    void setVerVelocity(double uVel) {
        cmd.data[1] = 2;
        cmd.data[8] = LimitValue(uVel, uVelSat);
        cmd.data[13] = 11;
    }
    void setYaw(double yawDeg) {
        cmd.data[2] = 1;
        cmd.data[9] = degreeRound0To360(yawDeg) * DEG2RAD_COE;
        cmd.data[13] = 11;
    }
    void setYawRate(double yawRateDeg) {
        cmd.data[2] = 3;
        cmd.data[9] = LimitValue(yawRateDeg, yawRateDegSat);
        cmd.data[13] = 11;
    }
    auto getCmd() {
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
    auto getPositionPositionYawCmd(double ePos, double nPos, double uPos, double yawDeg){
        cmd.data.resize(15, 0.0);
        setHorPosition(ePos, nPos);
        setVerPosition(uPos);
        setYaw(yawDeg);
        return getCmd();
    }
    auto getVelocityVelocityYawCmd(double eVel, double nVel, double uVel, double yawDeg){
        cmd.data.resize(15, 0.0);
        setHorVelocity(eVel, nVel);
        setVerVelocity(uVel);
        setYaw(yawDeg);
        return getCmd();
    }
    auto getVelocityVelocityYawRateCmd(double eVel, double nVel, double uVel, double yawRateDeg){
        cmd.data.resize(15, 0.0);
        setHorVelocity(eVel, nVel);
        setVerVelocity(uVel);
        setYawRate(yawRateDeg);
        return getCmd();
    }
    auto getAttitudeVelocityYawCmd(double pRad, double rRad, double uVel, double yawDeg){
        cmd.data.resize(15, 0.0);
        setHorAttitude(pRad, rRad);
        setVerVelocity(uVel);
        setYaw(yawDeg);
        return getCmd();
    }
    auto getSpinCmd(){
        cmd.data.resize(15, 0.0);
        cmd.data[13] = 22;
        cmd.data[14] = 9;
        return getCmd();
    }
    auto getTakeoffCmd(){
        cmd.data.resize(15, 0.0);
        cmd.data[13] = 22;
        cmd.data[14] = 1;
        return getCmd();
    }
    auto getLandCmd(){
        cmd.data.resize(15, 0.0);
        cmd.data[13] = 22;
        cmd.data[14] = 16;
        return getCmd();
    }
    auto getLockCmd(){
        cmd.data.resize(15, 0.0);
        cmd.data[13] = 22;
        cmd.data[14] = 40;
        return getCmd();
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

    Point currentRPYRad, currentRPYDeg;

    Point currentPosXY;

    Point currentPos;

    Point currentPosUWB;

    Point currentVelENU;

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
#ifdef GPS_POS
        setValue(currentPos, currentPosXY);
#endif

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
        setValue(currentPosUWB, msg->pose.pose.position);
#ifdef UWB_POS
        setValue(currentPos, currentPosUWB);
#endif
    }

    void setPositionOffset() {
#ifdef GPS_POS
        printf("GPS Position: %s", outputStr(fc.currentGPS).c_str());
#endif
#ifdef UWB_POS
        printf("UWB Position: %s", outputStr(fc.currentPosUWB).c_str());
#endif
        setValue(positionOffset, currentPos);
        printf("Position offset ENU / m: %s", outputStr(positionOffset).c_str());
    }

    Point compensatePositionOffset(Point _p){
        Point res;
#ifdef GPS_POS
        res.x = _p.x + positionOffset.x;
        res.y = _p.y + positionOffset.y;
#else
        res.x = _p.x;
        res.y = _p.y;
#endif
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

    bool yawNearDeg(double yawDeg, double tol=5.0) {
        return fabs(degreeRound(yawDeg - currentRPYDeg.z)) <= tol;
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
        ctrlCmdPub.publish(xyCmd.getVelocityVelocityYawCmd(0, 0, 0, 0));
    }

    void uavAdjustYaw(double _yawDeg) {
        ctrlCmdPub.publish(xyCmd.getVelocityVelocityYawCmd(0, 0, _yawDeg, 0));
    }

    void uavPositionPositionYawCtrl(double _x, double _y, double _z, double _yawDeg) {
        ctrlCmdPub.publish(xyCmd.getPositionPositionYawCmd(_x, _y, _z, _yawDeg));
    }

    void uavVelocityVelocityYawCtrl(double _vx, double _vy, double _vz, double _yawDeg) {
        ctrlCmdPub.publish(xyCmd.getVelocityVelocityYawCmd(_vx, _vy, _vz, _yawDeg));
    }

    void uavVelocityVelocityYawRateCtrl(double _vx, double _vy, double _vz, double _yawRateDeg) {
        ctrlCmdPub.publish(xyCmd.getVelocityVelocityYawRateCmd(_vx, _vy, _vz, _yawRateDeg));
    }

    void uavAttitudeVelocityYawCtrl(double _pRad, double _rRad, double _vz, double _yawDeg) {
        ctrlCmdPub.publish(xyCmd.getAttitudeVelocityYawCmd(_pRad, _rRad, _vz, _yawDeg));
    }

    template<typename T>
    void uavVelocityYawKPCtrl(T posDiff, double yawDeg){
        T vel;
        vel.x = posDiff.x * X_KP;
        vel.y = posDiff.y * Y_KP;
        vel.z = posDiff.z * Z_KP;
        uavVelocityVelocityYawCtrl(vel.x, vel.y, vel.z, yawDeg);
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
