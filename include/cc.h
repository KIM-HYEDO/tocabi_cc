#include "tocabi_lib/robot_data.h"
#include "wholebody_functions.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "sensor_msgs/JointState.h"
#include <tf/tf.h>
#include <tf2/LinearMath/Quaternion.h>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <random>
#include <fstream>
#include <ctime>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Bool.h>
#include <sys/stat.h>
#include <chrono>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Matrix3x3.h>
// #include <boost/shared_ptr.hpp>


class CustomController
{
public:
    CustomController(RobotData &rd);
    Eigen::VectorQd getControl();

    //void taskCommandToCC(TaskCommand tc_);
    
    void computeSlow();
    void computeFast();
    void computePlanner();
    void copyRobotData(RobotData &rd_l);
    void PublishHapticData();

    void DesRPoseCallback(const geometry_msgs::PoseStampedConstPtr &msg);
    void DesLPoseCallback(const geometry_msgs::PoseStampedConstPtr &msg);
    void DesHeadPoseCallback(const geometry_msgs::PoseStampedConstPtr &msg);
    void CupPosCallback(const geometry_msgs::PointPtr &msg);
    void JointTrajectoryCallback(const trajectory_msgs::JointTrajectoryPtr &msg);
    void JointTargetCallback(const sensor_msgs::JointStatePtr &msg);
    void JointReplayCallback(const sensor_msgs::JointStatePtr &msg);
    Eigen::Matrix3d Quat2rotmatrix(double q0, double q1, double q2, double q3);
    float PositionMapping( float haptic_pos, int i);
    bool saveImage(const sensor_msgs::ImageConstPtr &image_msg);
    void camera_img_callback(const sensor_msgs::ImageConstPtr &msg);
    void ReplayImgCallback(const sensor_msgs::ImageConstPtr &msg);
    // sensor_msgs::ImageConstPtr

    RobotData &rd_;
    RobotData rd_cc_;

    ros::NodeHandle nh_cc_;
    ros::CallbackQueue queue_cc_;
    ros::Subscriber des_r_pose_sub_;
    ros::Subscriber des_l_pose_sub_;
    ros::Subscriber des_head_pose_sub_;
    ros::Subscriber haptic_pose_sub_;
    ros::Subscriber joint_trajectory_sub;
    ros::Subscriber joint_target_sub;
    ros::Subscriber joint_replay_sub;
    ros::Publisher haptic_force_pub_;
    ros::Subscriber cup_pos_sub;
    
    bool des_r_subscribed=false;
    bool des_l_subscribed=false;
    bool des_head_subscribed=false;
    Eigen::Vector3d des_r_pos_;
    Eigen::Vector3d des_r_ori_;
    Eigen::Matrix3d des_r_orientation_;
    Eigen::Vector3d des_l_pos_;
    Eigen::Vector3d des_l_ori_;
    Eigen::Matrix3d des_l_orientation_;
    Eigen::Vector3d des_head_pos_;
    Eigen::Vector3d des_head_ori_;
    Eigen::Matrix3d des_head_orientation_;


    Eigen::Vector3d cup_pos_;
    Eigen::VectorQd desired_q_;
    Eigen::VectorQd desired_qdot_;
    std::vector<trajectory_msgs::JointTrajectoryPoint> points;
    bool init_time = false;
    int traj_index = -1;
    int num_waypoint = -1;

    bool target_received = false;
    double t_0_;
    std::vector<double> right_arm_target_first;
    std::vector<double> right_arm_target;
    bool first_rec=false;
    bool replay_image_start = false;
    Eigen::Matrix<double, 8, 1> q_0_;
    Eigen::Matrix<double, 8, 1> qdot_0_;
    Eigen::Matrix<double, 33, 1> q_0_33;
    Eigen::Matrix<double, 33, 1> qdot_0_33;
    int head_link[5] = {12,13,14,23,24};
    ros::Publisher terminate_pub;
    std_msgs::Bool terminate_msg;


    void resetRobotPose(double duration);
    bool target_reached_ = false;
    Eigen::Matrix<double, MODEL_DOF, 1> q_init_;
    double time_init_ = 0.0;
    
    std::string folderPath, filePath_hand, filePath_joint, filePath_info;   // for hand pose and joint
    std::string folderPath_image, fileName_image, filePath_image;           // for images
    std::ofstream fout, fout2, fout3;

    // float pos_x_;

    //WholebodyController &wbc_;
    //TaskCommand tc;

    double haptic_force_[3];

    float temp = 0.0;

    double rr_, rp_, ry_;
    double lr_, lp_, ly_;
    double drr_, drp_, dry_;
    double dlr_, dlp_, dly_;

    double hr_, hp_, hy_;
    double dhr_, dhp_, dhy_;
    Matrix3d hand_r_rot_desired_past;
    Matrix3d hand_l_rot_desired_past;

    Eigen::Matrix3d eulerRotation(double roll, double pitch, double yaw) {
    // Yaw, pitch, roll 순서로 회전 행렬 생성
    Eigen::Matrix3d rotation;
    rotation = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
               Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
               Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
    return rotation;
}


    ros::Publisher camera_flag_pub;
    std_msgs::Bool camera_flag_msg;

    ros::Publisher r_eef_pos_pub;
    ros::Publisher l_eef_pos_pub;
    ros::Publisher head_pos_pub;

    image_transport::Subscriber camera_image_sub;
    image_transport::Subscriber image_replay_sub;

    ros::Time last_received_time_;

    int camera_tick_ = 0;
    bool data_collect_start_ = false;
    bool make_dir = true;
    bool terminate = false;

    float distance_hand2obj;
    int prev_mode = 8;

    ros::Time timer;

private:
    Eigen::VectorQd ControlVal_;
};