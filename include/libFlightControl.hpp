#ifndef LIB_FLIGHT_CONTROL_HPP
#define LIB_FLIGHT_CONTROL_HPP

#include <math.h>
#include <time.h>

#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <queue>
#include <unistd.h>
#include <iostream>

#include <algorithm>

#include <geometry_msgs/PoseStamped.h>
#include <ros/duration.h>
#include <ros/ros.h>
#include <tf/tf.h>

#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "ros/time.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int8.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int8MultiArray.h"
#include "std_msgs/Empty.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include <cstring>

#include "geometry_msgs/Vector3.h"
typedef geometry_msgs::Vector3 Point;

#define PI std::acos(-1)
#define DEG2RAD_COE PI/180
#define RAD2DEG_COE 180/PI

#define RESET   "\033[0m"
#define BLACK   "\033[30m"
#define RED     "\033[31m"
#define GREEN   "\033[32m"
#define YELLOW  "\033[33m"
#define BLUE    "\033[34m"
#define MAGENTA "\033[35m"
#define CYAN    "\033[36m"
#define WHITE   "\033[37m"
#define BOLDBLACK   "\033[1m\033[30m"
#define BOLDRED     "\033[1m\033[31m"
#define BOLDGREEN   "\033[1m\033[32m"
#define BOLDYELLOW  "\033[1m\033[33m"
#define BOLDBLUE    "\033[1m\033[34m"
#define BOLDMAGENTA "\033[1m\033[35m"
#define BOLDCYAN    "\033[1m\033[36m"
#define BOLDWHITE   "\033[1m\033[37m"

namespace {
    struct DATA_STAT{
        int cnt;
        double mean, std, rms;
        DATA_STAT(){
            cnt = 0;
            mean = std = rms = 0.0;
        }
        void newData(double x){
            cnt++;
            std = pow(std, 2) / cnt * (cnt - 1) + (x - mean) * (x - mean) / cnt * (cnt - 1) / cnt;
            std = pow(std, 0.5);
            mean = mean / cnt * (cnt - 1) + x / cnt;
            rms = pow(rms, 2) / cnt * (cnt - 1) + x * x / cnt;
            rms = pow(rms, 0.5);
        }
    };

    struct MedianFilter{
        std::vector<double> v;
        size_t size;

        MedianFilter(size_t sz_){
            size = sz_;
            while (!v.empty()) v.erase(v.begin());
        }

        void newData(double nd_){
            v.push_back(nd_);
            while (v.size() > size) v.erase(v.begin());
        }

        double result(){
            std::vector<double> tmp = v;
            std::sort(tmp.begin(), tmp.end());
            return (tmp[(tmp.size() - 1) / 2] + tmp[tmp.size() / 2]) / 2;
        }

        void output(){
            printf("Now Median Filter Contains:");
            for (auto i: v) printf("\t%lf", i);
            printf("\n");
        }
    };

    struct AverageFilter{
        std::vector<double> v;
        size_t size;

        AverageFilter(size_t sz_){
            size = sz_;
            while (!v.empty()) v.erase(v.begin());
        }

        void newData(double nd_){
            v.push_back(nd_);
            while (v.size() > size) v.erase(v.begin());
        }

        double result(){
            std::vector<double> tmp = v;
            double res = 0.0;
            for (auto a: v){
                res += a;
            }
            return res / tmp.size();
        }

        void output(){
            printf("Now Average Filter Contains:");
            for (auto i: v) printf("\t%lf", i);
            printf("\n");
        }
    };

    template<typename T>
    struct XYZMedianFilter{
        MedianFilter x, y, z;
        XYZMedianFilter(int sz_ = 11): x(sz_), y(sz_), z(sz_){

        }

        void newData(T nd_){
            x.newData(nd_.x);
            y.newData(nd_.y);
            z.newData(nd_.z);
        }

        T result(){
            T res;
            res.x = x.result();
            res.y = y.result();
            res.z = z.result();
            return res;
        }

        void output(){
            x.output();
            y.output();
            z.output();
        }
    };

    template<typename T>
    struct XYZAverageFilter{
        AverageFilter x, y, z;
        XYZAverageFilter(int sz_ = 11): x(sz_), y(sz_), z(sz_){

        }

        void newData(T nd_){
            x.newData(nd_.x);
            y.newData(nd_.y);
            z.newData(nd_.z);
        }

        T result(){
            T res;
            res.x = x.result();
            res.y = y.result();
            res.z = z.result();
            return res;
        }

        void output(){
            x.output();
            y.output();
            z.output();
        }
    };

    void quaternion2Euler(double quat[4], double angle[3]){
        angle[0] = atan2(2.0 * (quat[3] * quat[2] + quat[0] * quat[1]), 1.0 - 2.0 * (quat[1] * quat[1] + quat[2] * quat[2]));
        angle[1] = asin(2.0 * (quat[2] * quat[0] - quat[3] * quat[1]));
        angle[2] = atan2(2.0 * (quat[3] * quat[0] + quat[1] * quat[2]), -1.0 + 2.0 * (quat[0] * quat[0] + quat[1] * quat[1]));
    }

    void Euler2Dcm(double euler[3], double dcm[3][3]){
        double phi = euler[0], theta = euler[1], psi = euler[2];
        double sinPhi = sin(phi), cosPhi = cos(phi);
        double sinThe = sin(theta), cosThe = cos(theta);
        double sinPsi = sin(psi), cosPsi = cos(psi);
        dcm[0][0] = cosThe * cosPsi;
        dcm[0][1] = cosThe * sinPsi;
        dcm[0][2] = -sinThe;

        dcm[1][0] = -cosPhi * sinPsi + sinPhi * sinThe * cosPsi;
        dcm[1][1] = cosPhi * cosPsi + sinPhi * sinThe * sinPsi;
        dcm[1][2] = sinPhi * cosThe;

        dcm[2][0] = sinPhi * sinPsi + cosPhi * sinThe * cosPsi;
        dcm[2][1] = -sinPhi * cosPsi + cosPhi * sinThe * sinPsi;
        dcm[2][2] = cosPhi * cosThe;
    }

    void dcmTranspose(double dcm[3][3], double dcmT[3][3]){
        for(size_t i=0;i<3;i++){
            for(size_t j=0;j<3;j++){
                dcmT[j][i] = dcm[i][j];
            }
        }
    }

    void matrixMultiply(double axis[3], double dcm[3][3], double axis_[3]){
        memset(axis_,0,sizeof(double)*3);
        for(size_t i=0;i<3;i++){
            for (size_t j=0;j<3;j++){
                axis_[i] += dcm[i][j] * axis[j];
            }
        }
    }

    double LimitValue(double x, double sat){
        return std::min(std::abs(sat), std::max(-std::abs(sat), x));
    }

    //solve target angle from zc
    void angleTransf(double euler[3], double bias, double pixel[2],double q[3]){
        //qAlpha: pitch->pixel[1]
        //qBeta:
        pixel[0] = -pixel[0];
        double theta = euler[1]+bias;
        double M1 = -cos(theta)*sin(euler[2])*cos(pixel[1])*cos(pixel[0])+(sin(theta)*sin(euler[2])*cos(euler[0])+cos(euler[2])*sin(euler[0]))*sin(pixel[1])-
                    (-sin(theta)*sin(euler[2])*sin(euler[0])+cos(euler[2])*cos(euler[0]))*cos(pixel[1])*sin(pixel[0]);
        double N1 = cos(theta)*cos(euler[2])*cos(pixel[1])*cos(pixel[0])+(-sin(theta)*cos(euler[2])*cos(euler[0])+sin(euler[2])*sin(euler[0]))*sin(pixel[1])-
                    (sin(theta)*cos(euler[2])*sin(euler[0])+sin(euler[2])*cos(euler[0]))*cos(pixel[1])*sin(pixel[0]);

        q[1] = asin(sin(theta)*cos(pixel[1])*cos(pixel[0])+cos(theta)*cos(euler[0])*sin(pixel[1])+cos(theta)*sin(euler[0])*cos(pixel[1])*sin(pixel[0]));
        q[2] = atan2(-M1,N1);
    }

    double degreeRound(double deg){
        if (deg < -180.0){
            return deg + 360.0;
        }
        if (deg > 180.0){
            return deg - 360.0;
        }
        return deg;
    }

    double degreeRound0To360(double deg){
        while (deg < 0.0){
            deg += 360.0;
        }
        while (deg >= 360.0){
            deg -= 360.0;
        }
        return deg;
    }

    double radRound(double rad){
        if (rad < -PI){
            return rad + 2 * PI;
        }
        if (rad > PI){
            return rad - 2 * PI;
        }
        return rad;
    }

    double nearlyIs(double a, double b, double tol = 0.1){
        return fabs(a - b) <= tol;
    }

    template<typename T>
    void e2b(T &a, double RE2b[3][3]){
        double xx = a.x, yy = a.y, zz = a.z;
        a.x = RE2b[0][0] * xx + RE2b[0][1] * yy + RE2b[0][2] * zz;
        a.y = RE2b[1][0] * xx + RE2b[1][1] * yy + RE2b[1][2] * zz;
        a.z = RE2b[2][0] * xx + RE2b[2][1] * yy + RE2b[2][2] * zz;
    }

    template<typename T>
    void b2e(T &a, double RE2b[3][3]){
        double xx = a.x, yy = a.y, zz = a.z;
        a.x = RE2b[0][0] * xx + RE2b[1][0] * yy + RE2b[2][0] * zz;
        a.y = RE2b[0][1] * xx + RE2b[1][1] * yy + RE2b[2][1] * zz;
        a.z = RE2b[0][2] * xx + RE2b[1][2] * yy + RE2b[2][2] * zz;
    }
    
    template<typename T1, typename T2>
    void setValue(T1 &a, T2 b){
        a.x = b.x;
        a.y = b.y;
        a.z = b.z;
    }

    template<typename T>
    void setValue(T &a, double b[]){
        a.x = b[0];
        a.y = b[1];
        a.z = b[2];
    }

    template<typename T>
    void setValue(double a[], T &b){
        a[0] = b.x;
        a[1] = b.y;
        a[2] = b.z;
    }

    template<typename T>
    void setValue(T &a, double x, double y, double z){
        a.x = x;
        a.y = y;
        a.z = z;
    }

    template<typename T1, typename T2>
    void setValueQuaternion(T1 &a, T2 b){
        a.x = b.x;
        a.y = b.y;
        a.z = b.z;
        a.w = b.w;
    }

    template<typename T1, typename T2>
    void saturateVel(T1 &a, T2 &b){
        a.x = LimitValue(a.x, b.x);
        a.y = LimitValue(a.y, b.y);
        a.z = LimitValue(a.z, b.z);
    }

    Point newPoint(double x, double y, double z){
        Point res;
        double p[3] = {x, y, z};
        setValue(res, p);
        return res;
    }

    Point newPoint(double p[]){
        Point res;
        setValue(res, p);
        return res;
    }

    template<typename T1, typename T2>
    double dis(T1 a, T2 b){
        return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2) + std::pow(a.z - b.z, 2));
    }

    template<typename T1, typename T2>
    double dissq(T1 a, T2 b){
        return std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2) + std::pow(a.z - b.z, 2);
    }

    template<typename T1, typename T2>
    double dis2d(T1 a, T2 b){
        return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
    }

    template<typename T1, typename T2>
    double angle2d(T1 from, T2 to){
        return atan2(to.y - from.y, to.x - from.x);
    }

    template<typename T1, typename T2>
    T1 minus(T1 a, T2 b){
        T1 res = a;
        res.x = a.x - b.x;
        res.y = a.y - b.y;
        res.z = a.z - b.z;
        return res;
    }

    template<typename T1, typename T2>
    T1 plus(T1 a, T2 b){
        T1 res = a;
        res.x = a.x + b.x;
        res.y = a.y + b.y;
        res.z = a.z + b.z;
        return res;
    }

    template<typename T>
    T scale(T a, double b){
        T res = a;
        res.x *= b;
        res.y *= b;
        res.z *= b;
        return res;
    }

    template<typename T>
    T interpolate(T a, T b, double ratio) {
        T res = a;
        res.x = a.x + (b.x - a.x) * ratio;
        res.y = a.y + (b.y - a.y) * ratio;
        res.z = a.z + (b.z - a.z) * ratio;
        return res;
    }

    template<typename T>
    double norm(T a){
        return std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
    }

    template<typename T>
    std::string outputStr(T a){
        char s[50];
        sprintf(s, "(%.2lf, %.2lf, %.2lf)", a.x, a.y, a.z);
        std::string res(s);
        return res;
    }

    template<typename T>
    void putDiscretePoints(std::vector<T> &v, T b, int n){
        assert(!v.empty());
        T a = v[v.size() - 1];
        for (int i = 1; i <= n; i++){
            v.push_back(plus(a, scale(minus(b, a), 1.0 * i / n)));
        }
    }

    template<typename T>
    void outputVector(std::vector<T> & v){
        for (auto i: v){
            printf("%s\n", output$1tr(i).c_str());
        }
    }

    uint8_t encodeUint8(uint32_t x, int n){
        return (x >> (8 * n)) & ((1 << 8) - 1);
    }

    uint32_t decodeUint8(std::vector<uint8_t> &v, int pos){
        return (v[pos] << 16) + (v[pos + 1] << 8) + v[pos + 2];
    }

    std::vector<Point> generateSmoothPath(Point start, Point end, int numPoints) {
        std::vector<Point> pathPoints;
        
        double stepX = (end.x - start.x) / (numPoints);
        double stepY = (end.y - start.y) / (numPoints);
        double stepZ = (end.z - start.z) / (numPoints);

        for (int i = 0; i < numPoints; ++i) {
            Point intermediatePoint;
            intermediatePoint.x = start.x + stepX * (i + 1);
            intermediatePoint.y = start.y + stepY * (i + 1);
            intermediatePoint.z = start.z + stepZ * (i + 1);
            pathPoints.push_back(intermediatePoint);
        }

        return pathPoints;
    }


}



#endif //LIB_FLIGHT_CONTROL_HPP
