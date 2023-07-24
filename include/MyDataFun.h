#ifndef MYDATAFUN_H
#define MYDATAFUN_H

#include "geometry_msgs/Vector3.h"
#include "MyMathFun.h"
#include <cstring>
#include <vector>

typedef geometry_msgs::Vector3 Point;


namespace MyDataFun{
    template<typename T1, typename T2>
    void set_value(T1 &a, T2 b){
        a.x = b.x;
        a.y = b.y;
        a.z = b.z;
    }

    template<typename T>
    void set_value(T &a, double b[]){
        a.x = b[0];
        a.y = b[1];
        a.z = b[2];
    }

    template<typename T>
    void set_value(double a[], T &b){
        a[0] = b.x;
        a[1] = b.y;
        a[2] = b.z;
    }

    template<typename T>
    void set_value(T &a, double x, double y, double z){
        a.x = x;
        a.y = y;
        a.z = z;
    }

    template<typename T1, typename T2>
    void set_value_quaternion(T1 &a, T2 b){
        a.x = b.x;
        a.y = b.y;
        a.z = b.z;
        a.w = b.w;
    }

    template<typename T1, typename T2>
    void saturate_vel(T1 &a, T2 &b){
        a.x = MyMathFun::LimitValue(a.x, b.x);
        a.y = MyMathFun::LimitValue(a.y, b.y);
        a.z = MyMathFun::LimitValue(a.z, b.z);
    }

    Point new_point(double x, double y, double z){
        Point res;
        double p[3] = {x, y, z};
        set_value(res, p);
        return res;
    }

    Point new_point(double p[]){
        Point res;
        set_value(res, p);
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
    double dis_2d(T1 a, T2 b){
        return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
    }

    template<typename T1, typename T2>
    double angle_2d(T1 from, T2 to){
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
    std::string output_str(T a){
        char s[50];
        sprintf(s, "(%.2lf, %.2lf, %.2lf)", a.x, a.y, a.z);
        std::string res(s);
        return res;
    }

    template<typename T>
    void put_discrete_points(std::vector<T> &v, T b, int n){
        assert(!v.empty());
        T a = v[v.size() - 1];
        for (int i = 1; i <= n; i++){
            v.push_back(plus(a, scale(minus(b, a), 1.0 * i / n)));
        }
    }

    template<typename T>
    void output_vector(std::vector<T> & v){
        for (auto i: v){
            printf("%s\n", output_str(i).c_str());
        }
    }

    uint8_t encode_uint8(uint32_t x, int n){
        return (x >> (8 * n)) & ((1 << 8) - 1);
    }

    uint32_t decode_uint8(std::vector<uint8_t> &v, int pos){
        return (v[pos] << 16) + (v[pos + 1] << 8) + v[pos + 2];
    }
}

#endif