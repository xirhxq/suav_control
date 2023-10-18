#ifndef MYMATHFUN_H
#define MYMATHFUN_H

#include <cmath>
#include <vector>

#define PI std::acos(-1)
#define DEG2RAD_COE PI/180
#define RAD2DEG_COE 180/PI

namespace MyMathFun{
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
}
	
#endif
