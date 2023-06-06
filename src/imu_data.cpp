#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv2/video.hpp>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <Eigen/Dense>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <mavros_msgs/PositionTarget.h>
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

            imu_full.angular_velocity.x = imu_raw_gyro[0]/180*3.1415926;
            imu_full.angular_velocity.y = imu_raw_gyro[1]/180*3.1415926;
            imu_full.angular_velocity.z = imu_raw_gyro[2]/180*3.1415926;//rad/s

            imu_full.linear_acceleration.x = -imu_raw_acc[0]*9.80665;
            imu_full.linear_acceleration.y = -imu_raw_acc[1]*9.80665;
            imu_full.linear_acceleration.z = -imu_raw_acc[2]*9.80665; //g

        }

        void imu_Callback(const sensor_msgs::ImuConstPtr& msg){
            vec[0] = msg->angular_velocity.x;
            vec[1] = msg->angular_velocity.y;
            vec[2] = msg->angular_velocity.z;
            q = Eigen::AngleAxisf(vec[0], ::Eigen::Vector3f::UnitZ()) *
            Eigen::AngleAxisf(vec[1], ::Eigen::Vector3f::UnitY()) *
            Eigen::AngleAxisf(vec[2], ::Eigen::Vector3f::UnitX());
            
            imu_full.orientation.x = q.x();
            imu_full.orientation.y = q.y();
            imu_full.orientation.z = q.z();
            imu_full.orientation.w = q.w();

            Matrix3f rx = q.toRotationMatrix();
            Eigen::Vector3f ea = rx.eulerAngles(2,1,0);      
        }

        void openvins_Callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg){
            Eigen::Quaternionf q;
            q.x() = msg->pose.pose.orientation.x;
            q.y() = msg->pose.pose.orientation.y;
            q.z() = msg->pose.pose.orientation.z;
            q.w() = msg->pose.pose.orientation.w;
            Matrix3f rx = q.toRotationMatrix();
            Eigen::Vector3f ea = rx.eulerAngles(2,1,0); 
            pos.position.x = msg->pose.pose.position.x;
            pos.position.y = msg->pose.pose.position.y;
            pos.position.z = msg->pose.pose.position.z;
            pos.yaw = ea[2];
            pos.type_mask = // mavros_msgs::PositionTarget::IGNORE_PX |
                                    // mavros_msgs::PositionTarget::IGNORE_PY |
                                    // mavros_msgs::PositionTarget::IGNORE_PZ |
                                    mavros_msgs::PositionTarget::IGNORE_VX |
                                    mavros_msgs::PositionTarget::IGNORE_VY |
                                    mavros_msgs::PositionTarget::IGNORE_VZ |
                                    mavros_msgs::PositionTarget::IGNORE_AFX |
                                    mavros_msgs::PositionTarget::IGNORE_AFY |
                                    mavros_msgs::PositionTarget::IGNORE_AFZ |
                                    mavros_msgs::PositionTarget::FORCE |
                                    // mavros_msgs::PositionTarget::IGNORE_YAW |
                                    mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
            pos.header = msg->header;  
        }
        
    
        void initial(){
            imu_mag_sub = n.subscribe<sensor_msgs::MagneticField>("/mavros/imu/mag", 140, &ImuConver::magCallback,this);
            imu_raw_sub = n.subscribe<sensor_msgs::Imu>("/mavros/imu/data_raw", 140, &ImuConver::imu_rawCallback,this);
            imu_sub = n.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 140, &ImuConver::imu_Callback,this);
            openvins_sub = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/ov_msckf/poseimu", 15, &ImuConver::openvins_Callback,this);
        }

        ImuConver(const ros::NodeHandle& nh):n(nh){

        }

    sensor_msgs::Imu imu_full;
    mavros_msgs::PositionTarget pos;

    private:
        ros::NodeHandle n;
        ros::Subscriber imu_mag_sub, imu_raw_sub, imu_sub, openvins_sub;
        float scale = 1;
        Vector3f imu_raw_gyro, imu_raw_acc;
        
        Quaternionf q;
        Vector3f vec;
        
};


int main(int argc, char *argv[]){
    ros::init(argc, argv, "imu_node");
    ros::NodeHandle nh("~");
    ImuConver imuConver(nh);
    ros::Publisher imu_full_pub = nh.advertise<sensor_msgs::Imu>("/mavros/imu/full",1);
    ros::Publisher setpoint_pub = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local",1);

    ros::Rate loop_rate(140);
    
    imuConver.initial();
    while (ros::ok()){
        imu_full_pub.publish(imuConver.imu_full);
        setpoint_pub.publish(imuConver.pos);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}