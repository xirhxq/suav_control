#include "FlightControl.hpp"    

int main(int argc, char** argv){
    ros::init(argc, argv, "pub_search_point");
    ros::NodeHandle nh;

    std::vector<double> uavPos1 = {300, 0, 40}, uavPos2 = {300, 0, 40};
    double uavYaw1 = 0.0, uavYaw2 = 0.0, uavYaw3 = 0.0;

    while(ros::ok()) {
        std::string userInupt;
        std::cout << "input point number: " << std::endl;
        std::cin >> userInupt;

        std_msgs::Float64MultiArray msg_point;
        msg_point.data.resize(6, 0.0);

        if(userInupt == "q") {
            break;
        }
        else if(userInupt == "0") {
            msg.data[0] = 0;
            msg.data[1] = uavPos1[0];
            msg.data[2] = uavPos1[1];
            msg.data[3] = uavPos1[2];
            msg.data[4] = 0.0;
            msg.data[5] = 60.0;
            searchPointPub.publish(msg_point);

            std::cout << "publish searchPoint[0]" << std::endl;
        }
        else if(userInupt == "1") {
            msg.data[0] = 1;
            msg.data[1] = uavPos1[0];
            msg.data[2] = uavPos1[1];
            msg.data[3] = uavPos1[2];
            msg.data[4] = uavYaw1;
            msg.data[5] = 60.0;
            searchPointPub.publish(msg_point);

            std::cout << "publish searchPoint[1]" << std::endl;
        }
        else if(userInupt == "2") {
            msg.data[0] = 2;
            msg.data[1] = uavPos2[0];
            msg.data[2] = uavPos2[1];
            msg.data[3] = uavPos2[2];
            msg.data[4] = uavYaw2;
            msg.data[5] = 60.0;
            searchPointPub.publish(msg_point);

            std::cout << "publish searchPoint[2]" << std::endl;
        }
        else if(userInupt == "3") {
            msg.data[0] = 3;
            msg.data[1] = uavPos1[0];
            msg.data[2] = uavPos1[1];
            msg.data[3] = uavPos1[2];
            msg.data[4] = uavYaw3;
            msg.data[5] = 60.0;
            searchPointPub.publish(msg);

            std::cout << "publish searchPoint[3]" << std::endl;
        }
        else if(userInupt == "6") {
            msg.data[0] = 6;
            msg.data[1] = uavPos1[0];
            msg.data[2] = uavPos1[1];
            msg.data[3] = uavPos1[2];
            msg.data[4] = 0.0;
            msg.data[5] = 60.0;
            searchPointPub.publish(msg_point);

            std::cout << "back home" << std::endl;
        }
        else {
            ROS_WARN("Invalid input");
        }

        ros::spinOnce();
        rate.sleep();
    }
}
