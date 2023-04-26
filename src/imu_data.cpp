#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv2/video.hpp>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <Eigen/Dense>
using namespace Eigen;
using namespace std;

class ImuConver{
    public:
        std_msgs::Header header;
        void magCallback(const sensor_msgs::MagneticFieldConstPtr& msg){
            scale = msg->magnetic_field.x;
        }

        void imu_rawCallback(const sensor_msgs::ImuConstPtr& msg){
            imu_raw_acc[0] = msg->linear_acceleration.x/scale*1.953125;
            imu_raw_acc[1] = msg->linear_acceleration.y/scale*1.953125;
            imu_raw_acc[2] = msg->linear_acceleration.z/scale*1.953125;

            imu_raw_gyro[0] = msg->angular_velocity.x * 17.4532925;
            imu_raw_gyro[1] = msg->angular_velocity.y * 17.4532925;
            imu_raw_gyro[2] = msg->angular_velocity.z * 17.4532925;

            imu_full.header = msg->header;

            imu_full.angular_velocity.x = imu_raw_gyro[0];
            imu_full.angular_velocity.y = imu_raw_gyro[1];
            imu_full.angular_velocity.z = imu_raw_gyro[2];//deg/s

            imu_full.linear_acceleration.x = imu_raw_acc[0];
            imu_full.linear_acceleration.y = imu_raw_acc[1];
            imu_full.linear_acceleration.z = imu_raw_acc[2]; //g

        }

        void imu_Callback(const sensor_msgs::ImuConstPtr& msg){
            imu_gyro[0] = msg->angular_velocity.x;
            imu_gyro[1] = msg->angular_velocity.y;
            imu_gyro[2] = msg->angular_velocity.z;
        }
        
    
        void initial(){
            imu_mag_sub = n.subscribe<sensor_msgs::MagneticField>("/mavros/imu/mag", 50, &ImuConver::magCallback,this);
            imu_raw_sub = n.subscribe<sensor_msgs::Imu>("/mavros/imu/data_raw", 50, &ImuConver::imu_rawCallback,this);
            imu_sub = n.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 50, &ImuConver::imu_Callback,this);
        }

        ImuConver(const ros::NodeHandle& nh):n(nh){

        }

    sensor_msgs::Imu imu_full;

    private:
        ros::NodeHandle n;
        ros::Subscriber imu_mag_sub, imu_raw_sub, imu_sub;
        float scale = 1;
        Vector3f imu_gyro, imu_raw_gyro, imu_raw_acc;
        
};


int main(int argc, char *argv[]){
    ros::init(argc, argv, "imu_node");
    ros::NodeHandle nh("~");
    ImuConver imuConver(nh);
    ros::Publisher imu_full_pub = nh.advertise<sensor_msgs::Imu>("/mavros/imu/full",1);
    
    ros::Rate loop_rate(100);
    
    imuConver.initial();
    while (ros::ok()){
        imu_full_pub.publish(imuConver.imu_full);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}