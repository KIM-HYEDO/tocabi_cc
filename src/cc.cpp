#include "cc.h"

std::default_random_engine generator;
ros::Publisher new_cup_pos_pub;
geometry_msgs::Point new_cup_pos_msg_;
geometry_msgs::PoseStamped r_eef_pos_msg_;
geometry_msgs::PoseStamped l_eef_pos_msg_;
geometry_msgs::PoseStamped head_pos_msg_;
ros::Time init;

using namespace TOCABI;


CustomController::CustomController(RobotData &rd) : rd_(rd) //, wbc_(dc.wbc_)
{
    nh_cc_.setCallbackQueue(&queue_cc_);
    
    des_r_pose_sub_ = nh_cc_.subscribe("/tocabi/r_posecommand", 1, &CustomController::DesRPoseCallback, this);
    des_l_pose_sub_ = nh_cc_.subscribe("/tocabi/l_posecommand", 1, &CustomController::DesLPoseCallback, this);
    des_head_pose_sub_ = nh_cc_.subscribe("/tocabi/headcommand", 1, &CustomController::DesHeadPoseCallback, this);
    cup_pos_sub = nh_cc_.subscribe("/cup_pos", 1, &CustomController::CupPosCallback, this);
    // joint_trajectory_sub = nh_cc_.subscribe("/tocabi/srmt/trajectory", 1, &CustomController::JointTrajectoryCallback, this);
    // joint_target_sub = nh_cc_.subscribe("/tocabi/act/joint_target", 1, &CustomController::JointTargetCallback, this);
    joint_replay_sub = nh_cc_.subscribe("/tocabi/jointstates_", 1, &CustomController::JointReplayCallback, this);
    
    ControlVal_.setZero();
    image_transport::ImageTransport it(nh_cc_);
    camera_flag_pub = nh_cc_.advertise<std_msgs::Bool>("/mujoco_ros_interface/camera/flag", 1);
    // camera_image_sub = it.subscribe("/mujoco_ros_interface/camera/image", 1, &CustomController::camera_img_callback, this);
    // image_replay_sub = it.subscribe("/image_raw", 1, &CustomController::ReplayImgCallback, this);
    new_cup_pos_pub = nh_cc_.advertise<geometry_msgs::Point>("/new_cup_pos", 1);
    terminate_pub = nh_cc_.advertise<std_msgs::Bool>("/tocabi/act/terminate", 1);
    r_eef_pos_pub = nh_cc_.advertise<geometry_msgs::PoseStamped>("/r_eef_pos", 1);
    l_eef_pos_pub = nh_cc_.advertise<geometry_msgs::PoseStamped>("/l_eef_pos", 1);
    head_pos_pub = nh_cc_.advertise<geometry_msgs::PoseStamped>("/head_pos", 1);


    // Kp_head
    // std::vector<double> Kp_r(6, 1.0);
    // nh_cc_.getParam("/Kp_r", Kp_r);
    // Kp_r_ << Kp_r[0], Kp_r[1], Kp_r[2], Kp_r[3], Kp_r[4], Kp_r[5];

    // std::vector<double> Kp_head(6, 1.0);
    // nh_cc_.getParam("/Kp_head", Kp_head);
    // Kp_head_ << Kp_head[0], Kp_head[1], Kp_head[2], Kp_head[3], Kp_head[4], Kp_head[5];


}

Eigen::VectorQd CustomController::getControl()
{
    return ControlVal_;
}

// void CustomController::taskCommandToCC(TaskCommand tc_)
// {
//     tc = tc_;
// }

double getRandomPosition(double minValue, double maxValue) 
{
    std::uniform_real_distribution<double> distribution(minValue, maxValue);
    return distribution(generator);
}

/*
bool CustomController::saveImage(const sensor_msgs::ImageConstPtr &image_msg) {
    cv::Mat image;
    // auto t_ = image_msg->header.stamp - init;
    auto t_ = ros::Time::now() - init;
    try
    {
      image = cv_bridge::toCvShare(image_msg, "bgr8")->image;
    }
    catch(cv_bridge::Exception)
    {
      ROS_ERROR("Unable to convert %s image to %s", image_msg->encoding.c_str(), "bgr8");
      return false;
    }

    if ((!image.empty()) && replay_image_start) {
        std::ostringstream oss;
        oss << std::setw(9) << std::setfill('0') << t_.nsec;
        std::stringstream fileNameSS;
        fileNameSS << "Image-" << camera_tick_ << "-" << t_.sec << "-" << oss.str() << ".jpg";
        fileName_image = fileNameSS.str();

        std::stringstream filePathSS;
        filePathSS << folderPath_image << "/" << fileName_image;
        filePath_image = filePathSS.str();

        cv::imwrite(filePath_image, image);
        // ROS_INFO("Saved image %s", fileName.c_str());
    }
    else
    {
        ROS_ERROR("Couldn't save image, no data!");
        return false;
    }

    return true;
}


void CustomController::camera_img_callback(const sensor_msgs::ImageConstPtr &msg)
{
    // if(data_collect_start_ || target_received){
    //     try
    //     {
    //         // ROS_INFO("Camera Callback. '%d'", camera_tick_);
    //         // save the image
    //         if (!saveImage(msg)) return;
    //         // ROS_INFO("Image Saved.");
    //     }
    //     catch (cv_bridge::Exception& e)
    //     {
    //         ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    //     }
    // }
}

void CustomController::ReplayImgCallback(const sensor_msgs::ImageConstPtr &msg)
{
    if(data_collect_start_ || target_received){
        try
        {
            // ROS_INFO("Camera Callback. '%d'", camera_tick_);
            // save the image
            if (!saveImage(msg)) return;
            // ROS_INFO("Image Saved.");
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        }
    }
}
*/

void CustomController::computeSlow()
{
    //MODE 6,7,8,9 is reserved for cc
    //MODE 6: joint target tracking for ACT
    //MODE 7: Homing & new cup pose
    //MODE 8: joint trajectory tracking for RRT & data collection
    //MODE 9: GUI end-effector pose tracking w/ HQP
    queue_cc_.callAvailable(ros::WallDuration());

    cnt_pub++;

    if (cnt_pub%2==0)
    {
        DyrosMath::rot2Euler_tf2(rd_.link_[Right_Hand].rotm, rr_, rp_, ry_);
        tf2::Quaternion r_eef_quat;
        r_eef_quat.setRPY(rr_, rp_, ry_);
        r_eef_pos_msg_.header.stamp = ros::Time::now();
        r_eef_pos_msg_.header.frame_id = "r_eef_pos";
        r_eef_pos_msg_.pose.position.x = rd_.link_[Right_Hand].xpos(0);
        r_eef_pos_msg_.pose.position.y = rd_.link_[Right_Hand].xpos(1);
        r_eef_pos_msg_.pose.position.z = rd_.link_[Right_Hand].xpos(2);
        r_eef_pos_msg_.pose.orientation = tf2::toMsg(r_eef_quat);
        r_eef_pos_pub.publish(r_eef_pos_msg_);

        DyrosMath::rot2Euler_tf2(rd_.link_[Left_Hand].rotm, lr_, lp_, ly_);
        tf2::Quaternion l_eef_quat;
        l_eef_quat.setRPY(lr_, lp_, ly_);
        l_eef_pos_msg_.header.stamp = ros::Time::now();
        l_eef_pos_msg_.header.frame_id = "l_eef_pos";
        l_eef_pos_msg_.pose.position.x = rd_.link_[Left_Hand].xpos(0);
        l_eef_pos_msg_.pose.position.y = rd_.link_[Left_Hand].xpos(1);
        l_eef_pos_msg_.pose.position.z = rd_.link_[Left_Hand].xpos(2);
        l_eef_pos_msg_.pose.orientation = tf2::toMsg(l_eef_quat);
        l_eef_pos_pub.publish(l_eef_pos_msg_);

        DyrosMath::rot2Euler_tf2(rd_.link_[Head].rotm, hr_, hp_, hy_);
        tf2::Quaternion head_quat;
        head_quat.setRPY(hr_, hp_, hy_);
        head_pos_msg_.header.stamp = ros::Time::now();
        head_pos_msg_.header.frame_id = "head_pos";
        head_pos_msg_.pose.position.x = rd_.link_[Head].xpos(0);
        head_pos_msg_.pose.position.y = rd_.link_[Head].xpos(1);
        head_pos_msg_.pose.position.z = rd_.link_[Head].xpos(2);
        head_pos_msg_.pose.orientation = tf2::toMsg(head_quat);
        head_pos_pub.publish(head_pos_msg_);
    }
    


    if (rd_.tc_.mode == 6) // replay data collect for whole body from sim
    {
        if (rd_.tc_init){
            std::cout << "mode 6 data collect!" << std::endl;
            rd_.tc_init = false;
            prev_mode = 6;
            first_rec = false;
            camera_tick_ = 0;
            make_dir = true;
            target_received = false;
            init = ros::Time::now();
            terminate_msg.data = false;
            terminate_pub.publish(terminate_msg);

            for(int i = 0; i < MODEL_DOF; i++){
                desired_q_[i] = rd_.q_[i];
                desired_qdot_[i] = 0.0;
            }
            // rd_.pos_kv_v[30] = 15; //25;
            // rd_.pos_kv_v[20] = 15; //25;
            replay_image_start = false;
        }
        if (target_received && !first_rec)
        {
            right_arm_target_first = right_arm_target;
            first_rec = true;
        }

        if(make_dir){
            std::cout<< "Make DIR" << std::endl;
            std::stringstream folderPathSS;
            auto now = std::chrono::system_clock::now();
            std::time_t currentTime = std::chrono::system_clock::to_time_t(now);
            folderPathSS << "/home/dyros/catkin_ws/src/tocabi/data/open_2arm/" << std::put_time(std::localtime(&currentTime), "%Y%m%d-%H%M%S");
            folderPath = folderPathSS.str();
            
            folderPathSS << "/image";
            folderPath_image = folderPathSS.str();

            int result1 = mkdir(folderPath.c_str(), 0777);
            int result2 = mkdir(folderPath_image.c_str(), 0777);
            if(result1 != 0){
                ROS_ERROR("Couldn't make folder(dir), Check folderPath");
            }
            if(result2 != 0){
                ROS_ERROR("Couldn't make folder2(dir), Check folderPath2");
            }
            make_dir = false;

            std::stringstream filePathSS_hand;
            filePathSS_hand << folderPath << "/hand.txt";
            filePath_hand = filePathSS_hand.str();
            fout.open(filePath_hand);
            fout << "camera_tick" << "\t" << "ros_time" << "\t" 
                    << "r_hand_pose_x" << "\t" << "r_hand_pose_y" << "\t" << "r_hand_pose_z" << "\t" 
                    << "r_hand_pose_roll" << "\t" << "r_hand_pose_pitch" << "\t" << "r_hand_pose_yaw" << "\t" 
                    << "l_hand_pose_x" << "\t" << "l_hand_pose_y" << "\t" << "l_hand_pose_z" << "\t" 
                    << "l_hand_pose_roll" << "\t" << "l_hand_pose_pitch" << "\t" << "l_hand_pose_yaw" << "\t" 
                    << "head_pose_x" << "\t" << "head_pose_y" << "\t" << "head_pose_z" << "\t" 
                    << "head_pose_roll" << "\t" << "head_pose_pitch" << "\t" << "head_pose_yaw" << endl; 
            if(!fout.is_open()){
                ROS_ERROR("Couldn't open text file");
            }

            std::stringstream filePathSS_joint;
            filePathSS_joint << folderPath << "/joint.txt";
            filePath_joint = filePathSS_joint.str();
            fout2.open(filePath_joint);
            fout2 << "camera_tick" << "\t" << "ros_time" << "\t" 
                    << "current_joint_pos" << "\t" << "target_joint_pos" << "\t" 
                    << "current_joint_vel" << "\t" << "target_joint_vel" << endl; 
            if(!fout2.is_open()){
                ROS_ERROR("Couldn't open text file2");
            }
        }
        
        if(target_received ){
            Eigen::Vector3d rhand_pos_;
            rhand_pos_ << rd_.link_[Right_Hand].xpos;

            Eigen::Vector3d lhand_pos_;
            lhand_pos_ << rd_.link_[Left_Hand].xpos;

            Eigen::Vector3d head_pos_;
            head_pos_ << rd_.link_[Head].xpos;

            WBC::SetContact(rd_, rd_.tc_.left_foot, rd_.tc_.right_foot, rd_.tc_.left_hand, rd_.tc_.right_hand);
            rd_.torque_grav = WBC::GravityCompensationTorque(rd_);

            if (init_time){
                traj_index = 0;
                init_time = false;
                init = ros::Time::now();
            }
            double t_ = (ros::Time::now() - init).toSec();

            for(int i = 0; i < MODEL_DOF; i++){
                desired_q_[i] = DyrosMath::cubic(t_, t_0_, t_0_+0.125, q_0_33[i], right_arm_target[i], qdot_0_33[i], 0);
                desired_qdot_[i] = DyrosMath::cubicDot(t_, t_0_, t_0_+0.125, q_0_33[i], right_arm_target[i], qdot_0_33[i], 0);
            }

            if((abs(right_arm_target_first[30]-right_arm_target[30])>0.0001) || replay_image_start) // data collect start trigger
            {
                if (!replay_image_start)
                {
                    init_time = true;
                    t_ = 0;
                    replay_image_start = true;
                } 
                
                // write data to the file
                if(camera_tick_%20 == 0){
                    DyrosMath::rot2Euler_tf2(rd_.link_[Right_Hand].rotm, rr_, rp_, ry_);
                    DyrosMath::rot2Euler_tf2(rd_.link_[Left_Hand].rotm, lr_, lp_, ly_);
                    DyrosMath::rot2Euler_tf2(rd_.link_[Head].rotm, hr_, hp_, hy_);
                    fout << camera_tick_ << "\t" << t_ << "\t" 
                         << rhand_pos_(0) << "\t" << rhand_pos_(1) << "\t" << rhand_pos_(2) << "\t" << rr_ << "\t"<< rp_ << "\t"<< ry_ << "\t"
                         << lhand_pos_(0) << "\t" << lhand_pos_(1) << "\t" << lhand_pos_(2) << "\t" << lr_ << "\t"<< lp_ << "\t"<< ly_ << "\t" 
                         << head_pos_(0)  << "\t" << head_pos_(1)  << "\t" << head_pos_(2)  << "\t" << hr_ << "\t"<< hp_ << "\t"<< hy_ << endl;
                }
                fout2 << camera_tick_ << "\t" << t_ << "\t";
                for(int i = 0; i < MODEL_DOF; i++){
                    fout2 << rd_.q_[i] << "\t";
                }
                for(int i = 0; i < MODEL_DOF; i++){
                    fout2 << desired_q_[i] << "\t";
                }
                for(int i = 0; i < MODEL_DOF; i++){
                    fout2 << rd_.q_dot_[i] << "\t";
                }
                for(int i = 0; i < MODEL_DOF; i++){
                    fout2 << desired_qdot_[i] << "\t";
                }
                fout2 << endl;
                camera_tick_++;
            }

            if ((ros::Time::now() - last_received_time_) > ros::Duration(2.0)) {
                std::cout<< "Stop!" << std::endl;
                terminate_msg.data = true;
                terminate_pub.publish(terminate_msg);
                rd_.tc_init = true;
                target_received=false;
                rd_.tc_.mode = 7;
            }
        }
        
        for (int i = 0; i < MODEL_DOF; i++){
            rd_.torque_desired[i] = rd_.pos_kp_v[i] * (desired_q_[i] - rd_.q_[i]) + rd_.pos_kv_v[i] * (desired_qdot_[i] - rd_.q_dot_[i]);
        }
    }

    if (rd_.tc_.mode == 6 && false) //data collect for arm from sim
    {
        if (rd_.tc_init){
            std::cout << "mode 6 init!" << std::endl;
            rd_.tc_init = false;
            prev_mode = 6;

            camera_tick_ = 0;
            make_dir = true;
            target_received = false;
            init = ros::Time::now();
            terminate_msg.data = false;
            terminate_pub.publish(terminate_msg);

            for(int i = 0; i < MODEL_DOF; i++){
                desired_q_[i] = rd_.q_[i];
                desired_qdot_[i] = 0.0;
            }
        }
        
        if(make_dir){
            std::stringstream folderPathSS;
            auto now = std::chrono::system_clock::now();
            std::time_t currentTime = std::chrono::system_clock::to_time_t(now);
            folderPathSS << "/home/dyros/catkin_ws/src/tocabi/result/" << std::put_time(std::localtime(&currentTime), "%Y%m%d-%H%M%S");
            folderPath = folderPathSS.str();
            
            folderPathSS << "/image";
            folderPath_image = folderPathSS.str();

            int result1 = mkdir(folderPath.c_str(), 0777);
            int result2 = mkdir(folderPath_image.c_str(), 0777);
            if(result1 != 0){
                ROS_ERROR("Couldn't make folder(dir), Check folderPath");
            }
            if(result2 != 0){
                ROS_ERROR("Couldn't make folder2(dir), Check folderPath2");
            }
            make_dir = false;

            std::stringstream filePathSS_hand;
            filePathSS_hand << folderPath << "/hand.txt";
            filePath_hand = filePathSS_hand.str();
            fout.open(filePath_hand);
            fout << "camera_tick" << "\t" << "ros_time" << "\t" << "hand_pose_x" << "\t" << "hand_pose_y" << "\t" << "hand_pose_z" << "\t" 
                    << "hand_pose_roll" << "\t" << "hand_pose_pitch" << "\t" << "hand_pose_yaw" << "\t" << "distance" << endl; 
            if(!fout.is_open()){
                ROS_ERROR("Couldn't open text file");
            }

            std::stringstream filePathSS_joint;
            filePathSS_joint << folderPath << "/joint.txt";
            filePath_joint = filePathSS_joint.str();
            fout2.open(filePath_joint);
            fout2 << "camera_tick" << "\t" << "ros_time" << "\t" 
                    << "current_joint_pos" << "\t" << "target_joint_pos" << "\t" 
                    << "current_joint_vel" << "\t" << "target_joint_vel" << endl; 
            if(!fout2.is_open()){
                ROS_ERROR("Couldn't open text file2");
            }

            std::stringstream filePathSS_info;
            filePathSS_info << folderPath << "/info.txt";
            filePath_info = filePathSS_info.str();
            fout3.open(filePath_info);
            fout3 << "cup_pose_x" << "\t" << "cup_pose_y" << "\t" << "cup_pose_z" << "\t" << "success" << endl;
            fout3 << cup_pos_(0) << "\t" << cup_pos_(1) << "\t" << cup_pos_(2) << "\t";
            if(!fout3.is_open()){
                ROS_ERROR("Couldn't open text file3");
            }
        }
            
        if(target_received){
            Eigen::Vector3d rhand_pos_;
            rhand_pos_ << rd_.link_[Right_Hand].xpos;
            
            float dx = cup_pos_(0) - rhand_pos_(0);
            float dy = cup_pos_(1) - rhand_pos_(1);
            float dz = cup_pos_(2) - rhand_pos_(2);
            distance_hand2obj = std::sqrt(dx*dx + dy*dy + dz*dz);

            WBC::SetContact(rd_, rd_.tc_.left_foot, rd_.tc_.right_foot, rd_.tc_.left_hand, rd_.tc_.right_hand);

            double t_ = (ros::Time::now() - init).toSec();
            for(int i = 0; i < 8; i++){
                desired_q_[MODEL_DOF-8+i] = DyrosMath::cubic(t_, t_0_, t_0_+0.125, q_0_[i], right_arm_target[i], qdot_0_[i], 0);
                desired_qdot_[MODEL_DOF-8+i] = DyrosMath::cubicDot(t_, t_0_, t_0_+0.125, q_0_[i], right_arm_target[i], qdot_0_[i], 0);
            }

            fout << camera_tick_ << "\t" << t_ << "\t" << rhand_pos_(0) << "\t" << rhand_pos_(1) << "\t" << rhand_pos_(2) << "\t" 
                    << rd_.link_[Right_Hand].roll << "\t"<< rd_.link_[Right_Hand].pitch << "\t"<< rd_.link_[Right_Hand].yaw << "\t" << distance_hand2obj << endl;
            
            fout2 << camera_tick_ << "\t" << t_ << "\t";
            for(int i = 0; i < MODEL_DOF; i++){
                fout2 << rd_.q_[i] << "\t";
            }
            for(int i = 0; i < MODEL_DOF; i++){
                fout2 << desired_q_[i] << "\t";
            }
            for(int i = 0; i < MODEL_DOF; i++){
                fout2 << rd_.q_dot_[i] << "\t";
            }
            for(int i = 0; i < MODEL_DOF; i++){
                fout2 << desired_qdot_[i] << "\t";
            }
            fout2 << endl;
            camera_tick_++;
            
            if (t_ > 20.0){
                bool is_reached = (distance_hand2obj < 0.3);
                fout3 << is_reached << endl;
                std::cout<< (is_reached ? "Reached" : "Failed") << std::endl;

                terminate_msg.data = true;
                terminate_pub.publish(terminate_msg);

                rd_.tc_.mode = 7;
                rd_.tc_init = true;
            }
        }
        for (int i = 0; i < MODEL_DOF; i++){
            rd_.torque_desired[i] = rd_.pos_kp_v[i] * (desired_q_[i] - rd_.q_[i]) + rd_.pos_kv_v[i] * (desired_qdot_[i] - rd_.q_dot_[i]);
        }
    }
    else if (rd_.tc_.mode == 7)
    {
        des_r_subscribed = false;
        des_l_subscribed = false;
        std::cout << "tc 7" << std::endl;
        if(fout.is_open()==true)
        {
            fout.close();
        }
        if(fout2.is_open()==true)
        {
            fout2.close();
        }
        if(fout3.is_open()==true)
        {
            fout3.close();
        }
        
        double duration = 2.0;
        if (!target_reached_)
        {
            target_reached_ = true;
            q_init_ = rd_.q_;
            time_init_ = rd_.control_time_;
        }
        resetRobotPose(duration);

        if (rd_.control_time_ > time_init_ + duration)
        {
            const double minX = -0.1;
            const double maxX = 0.1;
            const double minY = -0.3;
            const double maxY = 0.3;

            // new_cup_pos_msg_.x = getRandomPosition(minX, maxX);
            // new_cup_pos_msg_.y = getRandomPosition(minY, maxY);
            new_cup_pos_msg_.x = 0.0;
            new_cup_pos_msg_.y = 0.0;
            new_cup_pos_msg_.z = 0.0;
            new_cup_pos_pub.publish(new_cup_pos_msg_);

            // rd_.tc_.mode = prev_mode;
            target_reached_ = false;
        }
    }
    else if (rd_.tc_.mode == 8 && false)
    {
        // cout<<"MODE 8 START"<<endl;
        // initialize tc, executed only once
        if (rd_.tc_init){
            std::cout << "mode 8 init!" << std::endl;
            rd_.tc_init = false;
            prev_mode = 8;

            camera_tick_ = 0;
            make_dir = true;

            for(int i = 0; i < MODEL_DOF; i++){
                desired_q_[i] = rd_.q_[i];
                desired_qdot_[i] = 0.0;
            }
        }
        // initialize file for data collection
        if(make_dir){
            std::stringstream folderPathSS;
            // [Check] ros::Time::now().sec is int32 type.
            auto now = std::chrono::system_clock::now();
            std::time_t currentTime = std::chrono::system_clock::to_time_t(now);
            folderPathSS << "/home/dyros/catkin_ws/src/tocabi/data2/" << std::put_time(std::localtime(&currentTime), "%Y%m%d-%H%M%S");
            folderPath = folderPathSS.str();
            
            folderPathSS << "/image";
            folderPath_image = folderPathSS.str();

            int result1 = mkdir(folderPath.c_str(), 0777);
            int result2 = mkdir(folderPath_image.c_str(), 0777);
            if(result1 != 0){
                ROS_ERROR("Couldn't make folder(dir), Check folderPath");
            }
            if(result2 != 0){
                ROS_ERROR("Couldn't make folder2(dir), Check folderPath2");
            }
            make_dir = false;

            std::stringstream filePathSS_hand;
            filePathSS_hand << folderPath << "/hand.txt";
            filePath_hand = filePathSS_hand.str();
            fout.open(filePath_hand);
            fout << "camera_tick" << "\t" << "ros_time" << "\t" << "hand_pose_x" << "\t" << "hand_pose_y" << "\t" << "hand_pose_z" << "\t" 
                    << "hand_pose_roll" << "\t" << "hand_pose_pitch" << "\t" << "hand_pose_yaw" << "\t" << "distance" << endl; 
            if(!fout.is_open()){
                ROS_ERROR("Couldn't open text file");
            }

            std::stringstream filePathSS_joint;
            filePathSS_joint << folderPath << "/joint.txt";
            filePath_joint = filePathSS_joint.str();
            fout2.open(filePath_joint);
            fout2 << "camera_tick" << "\t" << "ros_time" << "\t" 
                    << "current_joint_pos" << "\t" << "target_joint_pos" << "\t" 
                    << "current_joint_vel" << "\t" << "target_joint_vel" << endl; 
            if(!fout2.is_open()){
                ROS_ERROR("Couldn't open text file2");
            }

            std::stringstream filePathSS_info;
            filePathSS_info << folderPath << "/info.txt";
            filePath_info = filePathSS_info.str();
            fout3.open(filePath_info);
            fout3 << "cup_pose_x" << "\t" << "cup_pose_y" << "\t" << "cup_pose_z" << "\t" << "success" << endl;
            fout3 << cup_pos_(0) << "\t" << cup_pos_(1) << "\t" << cup_pos_(2) << "\t";
            if(!fout3.is_open()){
                ROS_ERROR("Couldn't open text file3");
            }
        }
            
        if(data_collect_start_){
            // compute current distance between hand and cup for terminal condition check
            Eigen::Vector3d rhand_pos_;
            rhand_pos_ << rd_.link_[Right_Hand].xpos;
            
            float dx = cup_pos_(0) - rhand_pos_(0);
            float dy = cup_pos_(1) - rhand_pos_(1);
            float dz = cup_pos_(2) - rhand_pos_(2);
            distance_hand2obj = std::sqrt(dx*dx + dy*dy + dz*dz);

            WBC::SetContact(rd_, rd_.tc_.left_foot, rd_.tc_.right_foot, rd_.tc_.left_hand, rd_.tc_.right_hand);
            // rd_.torque_grav = WBC::GravityCompensationTorque(rd_);

            if (init_time){
                traj_index = 0;
                init_time = false;
                init = ros::Time::now();
            }
            double t_ = (ros::Time::now() - init).toSec();
            if (traj_index != -1){
                if (t_ > points[traj_index+1].time_from_start.toSec()){
                    traj_index++;
                }
                if (traj_index < num_waypoint-1){
                    double t_0 = points[traj_index].time_from_start.toSec();
                    auto   p_0 = points[traj_index].positions;
                    auto   v_0 = points[traj_index].velocities;
                    double t_f = points[traj_index+1].time_from_start.toSec();
                    auto   p_f = points[traj_index+1].positions;
                    auto   v_f = points[traj_index+1].velocities;
                    for(int i = 0; i < 8; i++){                    
                        desired_q_[MODEL_DOF-8+i] = DyrosMath::cubic(t_, t_0, t_f, p_0[i], p_f[i], v_0[i], v_f[i]);
                        desired_qdot_[MODEL_DOF-8+i] = DyrosMath::cubicDot(t_, t_0, t_f, p_0[i], p_f[i], v_0[i], v_f[i]);
                    }
                }
                else{
                    for(int i = 0; i < 8; i++){                    
                        desired_q_[MODEL_DOF-8+i] = points[traj_index].positions[i];
                        desired_qdot_[MODEL_DOF-8+i] = points[traj_index].velocities[i];
                    }
                }
            }

            // write data to the file
            if(camera_tick_%40 == 0){
                DyrosMath::rot2Euler_tf2(rd_.link_[Right_Hand].rotm, rr_, rp_, ry_);
                // ROS_INFO("I heard: [%d]", camera_tick_);
                camera_flag_msg.data = true;
                camera_flag_pub.publish(camera_flag_msg);
                fout << camera_tick_ << "\t" << t_ << "\t" << rhand_pos_(0) << "\t" << rhand_pos_(1) << "\t" << rhand_pos_(2) << "\t" 
                     << rr_ << "\t"<< rp_ << "\t"<< ry_ << "\t" << distance_hand2obj << endl;
            }
            fout2 << camera_tick_ << "\t" << t_ << "\t";
            for(int i = 0; i < MODEL_DOF; i++){
                fout2 << rd_.q_[i] << "\t";
            }
            for(int i = 0; i < MODEL_DOF; i++){
                fout2 << desired_q_[i] << "\t";
            }
            for(int i = 0; i < MODEL_DOF; i++){
                fout2 << rd_.q_dot_[i] << "\t";
            }
            for(int i = 0; i < MODEL_DOF; i++){
                fout2 << desired_qdot_[i] << "\t";
            }
            fout2 << endl;
            camera_tick_++;

            if (traj_index == num_waypoint-1){
                bool is_reached = (distance_hand2obj < 0.3);
                fout3 << is_reached << endl;
                std::cout<< (is_reached ? "Reached" : "Failed") << std::endl;
                data_collect_start_ = false;
                rd_.tc_.mode = 7;
                rd_.tc_init = true;
                traj_index = -1;
            }
        }
        for (int i = 0; i < MODEL_DOF; i++){
            rd_.torque_desired[i] = rd_.pos_kp_v[i] * (desired_q_[i] - rd_.q_[i]) + rd_.pos_kv_v[i] * (desired_qdot_[i] - rd_.q_dot_[i]);
        }
    }
    if (rd_.tc_.mode == 8 && des_r_subscribed && false) // only right arm
    {
        double timeStep = (ros::Time::now() - timer).toSec();

        timer = ros::Time::now();
        
        double ang2rad = 0.0174533;

        static bool init_qp;

        static Matrix3d r_rot_hand_init;
        static Matrix3d r_rot_des_init;

        static Matrix3d l_rot_hand_init;
        static Matrix3d l_rot_des_init;

        static Matrix3d rot_head_init;

        static Vector3d r_pos_hand_init;
        static Vector3d r_pos_des_init;

        static Vector3d l_pos_hand_init;
        static Vector3d l_pos_des_init;

        static Vector3d pos_head_init;

        if (rd_.tc_init)
        {
            init_qp = true;

            std::cout << "mode 8 init!" << std::endl;
            rd_.tc_init = false;
            rd_.link_[COM_id].x_desired = rd_.link_[COM_id].x_init;

            r_rot_hand_init = rd_.link_[Right_Hand].rotm;
            r_rot_des_init  = des_r_orientation_;
            l_rot_hand_init = rd_.link_[Left_Hand].rotm;
            l_rot_des_init  = des_l_orientation_;
            rot_head_init = rd_.link_[Head].rotm;
            r_pos_hand_init = rd_.link_[Right_Hand].xpos;
            r_pos_des_init= des_r_pos_;
            l_pos_hand_init = rd_.link_[Left_Hand].xpos;
            l_pos_des_init= des_l_pos_;
            pos_head_init = rd_.link_[Head].xpos;
            for(int i = 0; i < MODEL_DOF; i++){
                desired_q_[i] = rd_.q_[i];
                desired_qdot_[i] = 0.0;
            }
        }
        
        WBC::SetContact(rd_, rd_.tc_.left_foot, rd_.tc_.right_foot, rd_.tc_.left_hand, rd_.tc_.right_hand);
        if (rd_.tc_.customTaskGain)
        {
            // rd_.link_[Pelvis].SetGain(rd_.tc_.pos_p, rd_.tc_.pos_d, rd_.tc_.acc_p, rd_.tc_.ang_p, rd_.tc_.ang_d, 1);
            rd_.link_[Upper_Body].SetGain(rd_.tc_.pos_p, rd_.tc_.pos_d, rd_.tc_.acc_p, rd_.tc_.ang_p, rd_.tc_.ang_d, 1);
            rd_.link_[Right_Hand].SetGain(rd_.tc_.pos_p, rd_.tc_.pos_d, rd_.tc_.acc_p, rd_.tc_.ang_p, rd_.tc_.ang_d, 1);
        }
    

        // ARM // right ram jac
        // rd_.link_[Right_Hand].x_desired = des_r_pos_;
        // Vector3d hand_r_pos_desired = rd_.link_[Right_Hand].x_desired;
        Vector3d hand_r_pos_desired = des_r_pos_;
        Matrix3d hand_r_rot_desired = des_r_orientation_; 
        DyrosMath::rot2Euler_tf2(hand_r_rot_desired, drr_, drp_, dry_);

        Eigen::MatrixXd J = rd_.link_[Right_Hand].Jac();
        Eigen::MatrixXd J_r_arm = J.block(0, 6+33-8, 6, 8);
        Eigen::VectorXd r_pose_current(6); 
        r_pose_current << rd_.link_[Right_Hand].xpos(0), rd_.link_[Right_Hand].xpos(1), rd_.link_[Right_Hand].xpos(2), rr_, rp_, ry_;

        Eigen::VectorXd r_pose_desired(6); 
        r_pose_desired << hand_r_pos_desired(0), hand_r_pos_desired(1), hand_r_pos_desired(2), drr_, drp_, dry_;

        Eigen::VectorXd r_rot_diff(3); 
        r_rot_diff = DyrosMath::getPhi(rd_.link_[Right_Hand].rotm, hand_r_rot_desired);

        Eigen::VectorXd r_pose_error(6); 
        r_pose_error = r_pose_desired - r_pose_current;
        r_pose_error(3)=-r_rot_diff(0);
        r_pose_error(4)=-r_rot_diff(1);
        r_pose_error(5)=-r_rot_diff(2);

        Eigen::MatrixXd JtJ_r= J_r_arm.transpose() * J_r_arm;

        Eigen::VectorXd Kp_r_(6);
        Kp_r_ << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1; 

        Eigen::MatrixXd dq_r = JtJ_r.ldlt().solve(J_r_arm.transpose() * Kp_r_.cwiseProduct(r_pose_error));
        
        for(int i=0; i<8; i++)
        {
            desired_qdot_[MODEL_DOF-8+i] = dq_r(i);
            desired_q_[MODEL_DOF-8+i] += dq_r(i) * 0.0005;
        }
        for (int i = 0; i < MODEL_DOF; i++){
            rd_.torque_desired[i] = rd_.pos_kp_v[i] * (desired_q_[i] - rd_.q_[i]) + rd_.pos_kv_v[i] * (desired_qdot_[i] - rd_.q_dot_[i]);
        }

        init_qp = false;
    }
    if (rd_.tc_.mode == 8 && des_r_subscribed && des_head_subscribed) // head + right arm
    {
        double timeStep = (ros::Time::now() - timer).toSec();

        timer = ros::Time::now();
        
        double ang2rad = 0.0174533;

        static bool init_qp;

        static Matrix3d r_rot_hand_init;
        static Matrix3d r_rot_des_init;

        static Matrix3d l_rot_hand_init;
        static Matrix3d l_rot_des_init;

        static Matrix3d rot_head_init;

        static Vector3d r_pos_hand_init;
        static Vector3d r_pos_des_init;

        static Vector3d l_pos_hand_init;
        static Vector3d l_pos_des_init;

        static Vector3d pos_head_init;

        double alpha = 0.1; // 필터 계수 (조정 가능)
        Eigen::VectorXd filtered_qdot_(MODEL_DOF);
        Eigen::VectorXd filtered_q_(MODEL_DOF);

        if (rd_.tc_init)
        {
            init_qp = true;

            std::cout << "mode 9 init!" << std::endl;
            rd_.tc_init = false;
            rd_.link_[COM_id].x_desired = rd_.link_[COM_id].x_init;

            r_rot_hand_init = rd_.link_[Right_Hand].rotm;
            r_rot_des_init  = des_r_orientation_;
            l_rot_hand_init = rd_.link_[Left_Hand].rotm;
            l_rot_des_init  = des_l_orientation_;
            rot_head_init = rd_.link_[Head].rotm;
            r_pos_hand_init = rd_.link_[Right_Hand].xpos;
            r_pos_des_init= des_r_pos_;
            l_pos_hand_init = rd_.link_[Left_Hand].xpos;
            l_pos_des_init= des_l_pos_;
            pos_head_init = rd_.link_[Head].xpos;
            for(int i = 0; i < MODEL_DOF; i++){
                desired_q_[i] = rd_.q_[i];
                desired_qdot_[i] = 0.0;
            }
        }
        
        WBC::SetContact(rd_, 1, 1);
        // if (rd_.tc_.customTaskGain)
        // {
        //     // rd_.link_[Pelvis].SetGain(rd_.tc_.pos_p, rd_.tc_.pos_d, rd_.tc_.acc_p, rd_.tc_.ang_p, rd_.tc_.ang_d, 1);
        //     rd_.link_[Upper_Body].SetGain(rd_.tc_.pos_p, rd_.tc_.pos_d, rd_.tc_.acc_p, rd_.tc_.ang_p, rd_.tc_.ang_d, 1);
        //     rd_.link_[Right_Hand].SetGain(rd_.tc_.pos_p, rd_.tc_.pos_d, rd_.tc_.acc_p, rd_.tc_.ang_p, rd_.tc_.ang_d, 1);
        // }
        
        // HEAD // 머리쪽 자코비안
        // rd_.link_[Head].x_desired = pos_head_init;
        // rd_.link_[Head].x_desired = des_head_pos_;
        // Vector3d head_pos_desired = rd_.link_[Head].x_desired;
        Vector3d head_pos_desired = des_head_pos_;
        // Matrix3d head_rot_desired = rot_head_init;
        Matrix3d head_rot_desired = des_head_orientation_;
        DyrosMath::rot2Euler_tf2(head_rot_desired, dhr_, dhp_, dhy_);

        Eigen::MatrixXd J = rd_.link_[Head].Jac();
        Eigen::MatrixXd J_head(6, 5); 
        J_head.block(0, 0, 6, 3) = J.block(0, 6+12, 6, 3);
        J_head.block(0, 3, 6, 2) = J.block(0, 6+23, 6, 2);

        Eigen::VectorXd pose_head_current(6); 
        pose_head_current << rd_.link_[Head].xpos(0), rd_.link_[Head].xpos(1), rd_.link_[Head].xpos(2), hr_, hp_, hy_;
        
        Eigen::VectorXd pose_head_desired(6); 
        pose_head_desired << head_pos_desired(0), head_pos_desired(1), head_pos_desired(2), dhr_, dhp_, dhy_;

        Eigen::VectorXd rot_head_diff(3); 
        rot_head_diff = DyrosMath::getPhi(rd_.link_[Head].rotm, head_rot_desired);

        Eigen::VectorXd pose_head_error(6); 
        pose_head_error = pose_head_desired - pose_head_current;
        pose_head_error(3)=-rot_head_diff(0);
        pose_head_error(4)=-rot_head_diff(1);
        pose_head_error(5)=-rot_head_diff(2);

        Eigen::MatrixXd JtJ_head= J_head.transpose() * J_head;

        Eigen::VectorXd Kp_head_(6);
        // Kp_head_ << 2.0, 2.0, 2.0, 2.0, 2.0, 2.0; 
        Kp_head_ <<1.5, 2.0, 2.0, 2.0, 2.0, 2.0; 

        Eigen::MatrixXd dq_head = JtJ_head.ldlt().solve(J_head.transpose() * Kp_head_.cwiseProduct(pose_head_error));
        
        for(int i=0; i<5; i++)
        {
            desired_qdot_[head_link[i]] = dq_head(i);
            desired_q_[head_link[i]] += dq_head(i) * 0.0005;
        }

        // ARM // right ram jac
        // rd_.link_[Right_Hand].x_desired = des_r_pos_;
        // Vector3d hand_r_pos_desired = rd_.link_[Right_Hand].x_desired;
        Vector3d hand_r_pos_desired = des_r_pos_;
        Matrix3d hand_r_rot_desired = des_r_orientation_; 
        DyrosMath::rot2Euler_tf2(hand_r_rot_desired, drr_, drp_, dry_);

        J = rd_.link_[Right_Hand].Jac();
        Eigen::MatrixXd J_r_arm = J.block(0, 6+33-8, 6, 8);
        Eigen::VectorXd r_pose_current(6); 
        r_pose_current << rd_.link_[Right_Hand].xpos(0), rd_.link_[Right_Hand].xpos(1), rd_.link_[Right_Hand].xpos(2), rr_, rp_, ry_;

        Eigen::VectorXd r_pose_desired(6); 
        r_pose_desired << hand_r_pos_desired(0), hand_r_pos_desired(1), hand_r_pos_desired(2), drr_, drp_, dry_;

        Eigen::VectorXd r_rot_diff(3); 
        r_rot_diff = DyrosMath::getPhi(rd_.link_[Right_Hand].rotm, hand_r_rot_desired);

        Eigen::VectorXd r_pose_error(6); 
        r_pose_error = r_pose_desired - r_pose_current;
        r_pose_error(3)=-r_rot_diff(0);
        r_pose_error(4)=-r_rot_diff(1);
        r_pose_error(5)=-r_rot_diff(2);

        Eigen::MatrixXd JtJ_r= J_r_arm.transpose() * J_r_arm;

        Eigen::VectorXd Kp_r_(6);
        // Kp_r_ << 2.0, 2.0, 2.0, 1.5, 1.5, 1.5; 
        Kp_r_ << 1.5, 2.0, 2.0, 1.5, 1.5, 1.5; 

        Eigen::MatrixXd dq_r = JtJ_r.ldlt().solve(J_r_arm.transpose() * Kp_r_.cwiseProduct(r_pose_error));

       
        
        for(int i=0; i<8; i++)
        {
            desired_qdot_[MODEL_DOF-8+i] = dq_r(i);
            desired_q_[MODEL_DOF-8+i] += dq_r(i) * 0.0005;
        }

        // for (int i = 0; i < MODEL_DOF; i++){
        //     rd_.torque_desired[i] = rd_.pos_kp_v[i] * (desired_q_[i] - rd_.q_[i]) + rd_.pos_kv_v[i] * (desired_qdot_[i] - rd_.q_dot_[i]);
        // }

        // 필터 초기값 설정
        for (int i = 0; i < MODEL_DOF; i++) {
            filtered_qdot_[i] = desired_qdot_[i];
            filtered_q_[i] = desired_q_[i];
        }

        // 기존 코드 ...
        for (int i = 0; i < 8; i++)
        {
            desired_qdot_[MODEL_DOF - 8 + i] = dq_r(i);
            // 필터 적용
            filtered_qdot_[MODEL_DOF - 8 + i] = alpha * desired_qdot_[MODEL_DOF - 8 + i] + (1 - alpha) * filtered_qdot_[MODEL_DOF - 8 + i];
            
            desired_q_[MODEL_DOF - 8 + i] += dq_r(i) * 0.0005;
            // 필터 적용
            filtered_q_[MODEL_DOF - 8 + i] = alpha * desired_q_[MODEL_DOF - 8 + i] + (1 - alpha) * filtered_q_[MODEL_DOF - 8 + i];
        }

        for (int i = 0; i < 5; i++)
        {
            desired_qdot_[head_link[i]] = dq_head(i);
            // 필터 적용
            filtered_qdot_[head_link[i]] = alpha * desired_qdot_[head_link[i]] + (1 - alpha) * filtered_qdot_[head_link[i]];
            
            desired_q_[head_link[i]] += dq_head(i) * 0.0005;
            // 필터 적용
            filtered_q_[head_link[i]] = alpha * desired_q_[head_link[i]] + (1 - alpha) * filtered_q_[head_link[i]];
        }

        // 나머지 코드 ...
        for (int i = 0; i < MODEL_DOF; i++){
            rd_.torque_desired[i] = rd_.pos_kp_v[i] * (filtered_q_[i] - rd_.q_[i]) + rd_.pos_kv_v[i] * (filtered_qdot_[i] - rd_.q_dot_[i]);
        }

        init_qp = false;
    }

    if (rd_.tc_.mode == 9 && des_r_subscribed && des_l_subscribed && des_head_subscribed && false) // head + 2-arms
    {
        double timeStep = (ros::Time::now() - timer).toSec();

        timer = ros::Time::now();
        
        double ang2rad = 0.0174533;

        static bool init_qp;

        static Matrix3d r_rot_hand_init;
        static Matrix3d r_rot_des_init;

        static Matrix3d l_rot_hand_init;
        static Matrix3d l_rot_des_init;

        static Matrix3d rot_head_init;

        static Vector3d r_pos_hand_init;
        static Vector3d r_pos_des_init;

        static Vector3d l_pos_hand_init;
        static Vector3d l_pos_des_init;

        static Vector3d pos_head_init;

        if (rd_.tc_init)
        {
            init_qp = true;

            std::cout << "mode 9 init!" << std::endl;
            rd_.tc_init = false;
            rd_.link_[COM_id].x_desired = rd_.link_[COM_id].x_init;

            r_rot_hand_init = rd_.link_[Right_Hand].rotm;
            r_rot_des_init  = des_r_orientation_;
            l_rot_hand_init = rd_.link_[Left_Hand].rotm;
            l_rot_des_init  = des_l_orientation_;
            rot_head_init = rd_.link_[Head].rotm;
            r_pos_hand_init = rd_.link_[Right_Hand].xpos;
            r_pos_des_init= des_r_pos_;
            l_pos_hand_init = rd_.link_[Left_Hand].xpos;
            l_pos_des_init= des_l_pos_;
            pos_head_init = rd_.link_[Head].xpos;
            for(int i = 0; i < MODEL_DOF; i++){
                desired_q_[i] = rd_.q_[i];
                desired_qdot_[i] = 0.0;
            }
        }
        
        WBC::SetContact(rd_, rd_.tc_.left_foot, rd_.tc_.right_foot, rd_.tc_.left_hand, rd_.tc_.right_hand);
        if (rd_.tc_.customTaskGain)
        {
            rd_.link_[Pelvis].SetGain(rd_.tc_.pos_p, rd_.tc_.pos_d, rd_.tc_.acc_p, rd_.tc_.ang_p, rd_.tc_.ang_d, 1);
            rd_.link_[Upper_Body].SetGain(rd_.tc_.pos_p, rd_.tc_.pos_d, rd_.tc_.acc_p, rd_.tc_.ang_p, rd_.tc_.ang_d, 1);
            rd_.link_[Right_Hand].SetGain(rd_.tc_.pos_p, rd_.tc_.pos_d, rd_.tc_.acc_p, rd_.tc_.ang_p, rd_.tc_.ang_d, 1);
        }
        
        // HEAD // 머리쪽 자코비안
        // rd_.link_[Head].x_desired = pos_head_init;
        rd_.link_[Head].x_desired = des_head_pos_;
        Vector3d head_pos_desired = rd_.link_[Head].x_desired;
        // Matrix3d head_rot_desired = rot_head_init;
        Matrix3d head_rot_desired = des_head_orientation_;
        DyrosMath::rot2Euler_tf2(head_rot_desired, dhr_, dhp_, dhy_);

        Eigen::MatrixXd J = rd_.link_[Head].Jac();
        Eigen::MatrixXd J_head(6, 5); 
        J_head.block(0, 0, 6, 3) = J.block(0, 6+12, 6, 3);
        J_head.block(0, 3, 6, 2) = J.block(0, 6+23, 6, 2);

        Eigen::VectorXd pose_head_current(6); 
        pose_head_current << rd_.link_[Head].xpos(0), rd_.link_[Head].xpos(1), rd_.link_[Head].xpos(2), hr_, hp_, hy_;
        
        Eigen::VectorXd pose_head_desired(6); 
        pose_head_desired << head_pos_desired(0), head_pos_desired(1), head_pos_desired(2), dhr_, dhp_, dhy_;

        Eigen::VectorXd rot_head_diff(3); 
        rot_head_diff = DyrosMath::getPhi(rd_.link_[Head].rotm, head_rot_desired);

        Eigen::VectorXd pose_head_error(6); 
        pose_head_error = pose_head_desired - pose_head_current;
        pose_head_error(3)=-rot_head_diff(0);
        pose_head_error(4)=-rot_head_diff(1);
        pose_head_error(5)=-rot_head_diff(2);

        Eigen::MatrixXd JtJ_head= J_head.transpose() * J_head;

        Eigen::VectorXd Kp_head(6);
        Kp_head << 1, 1, 1, 1, 1, 1; 

        Eigen::MatrixXd dq_head = JtJ_head.ldlt().solve(J_head.transpose() * Kp_head.cwiseProduct(pose_head_error));
        
        for(int i=0; i<5; i++)
        {
            desired_qdot_[head_link[i]] = dq_head(i);
            desired_q_[head_link[i]] += dq_head(i) * 0.0005;
        }

        // ARM // right ram jac
        rd_.link_[Right_Hand].x_desired = des_r_pos_;
        Vector3d hand_r_pos_desired = rd_.link_[Right_Hand].x_desired;
        Matrix3d hand_r_rot_desired = des_r_orientation_; 
        DyrosMath::rot2Euler_tf2(hand_r_rot_desired, drr_, drp_, dry_);

        J = rd_.link_[Right_Hand].Jac();
        Eigen::MatrixXd J_r_arm = J.block(0, 6+33-8, 6, 8);
        Eigen::VectorXd r_pose_current(6); 
        r_pose_current << rd_.link_[Right_Hand].xpos(0), rd_.link_[Right_Hand].xpos(1), rd_.link_[Right_Hand].xpos(2), rr_, rp_, ry_;

        Eigen::VectorXd r_pose_desired(6); 
        r_pose_desired << hand_r_pos_desired(0), hand_r_pos_desired(1), hand_r_pos_desired(2), drr_, drp_, dry_;

        Eigen::VectorXd r_rot_diff(3); 
        r_rot_diff = DyrosMath::getPhi(rd_.link_[Right_Hand].rotm, hand_r_rot_desired);

        Eigen::VectorXd r_pose_error(6); 
        r_pose_error = r_pose_desired - r_pose_current;
        r_pose_error(3)=-r_rot_diff(0);
        r_pose_error(4)=-r_rot_diff(1);
        r_pose_error(5)=-r_rot_diff(2);

        Eigen::MatrixXd JtJ_r= J_r_arm.transpose() * J_r_arm;

        Eigen::VectorXd Kp(6);
        Kp << 1, 1, 1, 1, 1, 1; 

        Eigen::MatrixXd dq_r = JtJ_r.ldlt().solve(J_r_arm.transpose() * Kp.cwiseProduct(r_pose_error));
        
        for(int i=0; i<8; i++)
        {
            desired_qdot_[MODEL_DOF-8+i] = dq_r(i);
            desired_q_[MODEL_DOF-8+i] += dq_r(i) * 0.0005;
        }
        for (int i = 0; i < MODEL_DOF; i++){
            rd_.torque_desired[i] = rd_.pos_kp_v[i] * (desired_q_[i] - rd_.q_[i]) + rd_.pos_kv_v[i] * (desired_qdot_[i] - rd_.q_dot_[i]);
        }

        init_qp = false;
    }
}

void CustomController::resetRobotPose(double duration)
{
    Eigen::Matrix<double, MODEL_DOF, 1> q_target;
    Eigen::Matrix<double, MODEL_DOF, 1> q_cubic;

    // Init Pos
    // q_target << 0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
    //         0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
    //         0.0, 0.0, 0.0,
    //         0.3, 0.3, 1.5, -1.27, -1.0, 0.0, -1.0, 0.0,
    //         0.0, 0.0,
    //         -0.3, -0.9, -1.5, 1.57, 1.9, 0.0, 0.5, 0.0;

    // Ready Pos
    q_target << 0.0, 0.0, -0.225, 0.564227, -0.339828, 0.0,
            0.0, 0.0, -0.225378, 0.564208, -0.339865, 0.0,
            0.000299952, -0.000234845, -0.00196816,
            0.0178709, -0.257828, 1.5481, -1.21629, -1.5417, 1.48712, 0.368517, -0.171078,
            -0.00140207, 0.292029,
            -0.0178855, 0.250205, -1.54612, 1.21486, 1.55032, -1.48743, -0.369572, 0.188141;

    for (int i = 0; i <MODEL_DOF; i++)
    {
        q_cubic(i) = DyrosMath::cubic(rd_.control_time_, time_init_, time_init_ +duration, q_init_(i), q_target(i), 0.0, 0.0);
    }
    Eigen::Matrix<double, MODEL_DOF, MODEL_DOF> kp;
    Eigen::Matrix<double, MODEL_DOF, MODEL_DOF> kv;

    kp.setZero();
    kv.setZero();
    kp.diagonal() <<    2000.0, 5000.0, 4000.0, 3700.0, 3200.0, 3200.0,
                        2000.0, 5000.0, 4000.0, 3700.0, 3200.0, 3200.0,
                        6000.0, 10000.0, 10000.0,
                        400.0, 1000.0, 400.0, 400.0, 400.0, 400.0, 100.0, 100.0,
                        100.0, 100.0,
                        400.0, 1000.0, 400.0, 400.0, 400.0, 400.0, 100.0, 100.0;
    kv.diagonal() <<    15.0, 50.0, 20.0, 25.0, 24.0, 24.0,
                        15.0, 50.0, 20.0, 25.0, 24.0, 24.0,
                        200.0, 100.0, 100.0,
                        10.0, 28.0, 10.0, 10.0, 10.0, 10.0, 3.0, 3.0,
                        2.0, 2.0,
                        10.0, 28.0, 10.0, 10.0, 10.0, 10.0, 3.0, 3.0;


    WBC::SetContact(rd_, rd_.tc_.left_foot, rd_.tc_.right_foot, rd_.tc_.left_hand, rd_.tc_.right_hand);
    rd_.torque_desired =  kp*(q_cubic-rd_.q_)-kv* rd_.q_dot_ +  WBC::GravityCompensationTorque(rd_);
}


void CustomController::computeFast()
{
    if (rd_.tc_.mode == 6)
    {
    }
    else if (rd_.tc_.mode == 7)
    {
    }
}

void CustomController::DesRPoseCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
    float pos_x = msg -> pose.position.x;
    float pos_y = msg -> pose.position.y;
    float pos_z = msg -> pose.position.z;
    float ori_x = msg -> pose.orientation.x;
    float ori_y = msg -> pose.orientation.y;
    float ori_z = msg -> pose.orientation.z;
    float ori_w = msg -> pose.orientation.w;

    des_r_pos_[0] = pos_x;
    des_r_pos_[1] = pos_y; 
    des_r_pos_[2] = pos_z;
    
    tf2::Quaternion quat(ori_x, ori_y, ori_z, ori_w);

    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    des_r_ori_[0] = roll;
    des_r_ori_[1] = pitch;
    des_r_ori_[2] = yaw;

    des_r_orientation_ = eulerRotation(roll, pitch, yaw);
    des_r_subscribed = true;
}

void CustomController::DesLPoseCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
    float pos_x = msg -> pose.position.x;
    float pos_y = msg -> pose.position.y;
    float pos_z = msg -> pose.position.z;
    float ori_x = msg -> pose.orientation.x;
    float ori_y = msg -> pose.orientation.y;
    float ori_z = msg -> pose.orientation.z;
    float ori_w = msg -> pose.orientation.w;

    des_l_pos_[0] = pos_x;
    des_l_pos_[1] = pos_y; 
    des_l_pos_[2] = pos_z;
    
    tf2::Quaternion quat(ori_x, ori_y, ori_z, ori_w);

    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    des_l_ori_[0] = roll;
    des_l_ori_[1] = pitch;
    des_l_ori_[2] = yaw;

    des_l_orientation_ = eulerRotation(roll, pitch, yaw);
    des_l_subscribed = true;
}

void CustomController::DesHeadPoseCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
    float pos_x = msg -> pose.position.x;
    float pos_y = msg -> pose.position.y;
    float pos_z = msg -> pose.position.z;
    float ori_x = msg -> pose.orientation.x;
    float ori_y = msg -> pose.orientation.y;
    float ori_z = msg -> pose.orientation.z;
    float ori_w = msg -> pose.orientation.w;

    des_head_pos_[0] = pos_x;
    des_head_pos_[1] = pos_y; 
    des_head_pos_[2] = pos_z;
    
    tf2::Quaternion quat(ori_x, ori_y, ori_z, ori_w);

    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    des_head_ori_[0] = roll;
    des_head_ori_[1] = pitch;
    des_head_ori_[2] = yaw;

    des_head_orientation_ = eulerRotation(roll, pitch, yaw);
    des_head_subscribed = true;
}

void CustomController::CupPosCallback(const geometry_msgs::PointPtr &msg)
{
    float cup_x = msg -> x;
    float cup_y = msg -> y;
    float cup_z = msg -> z;
 
    cup_pos_ << cup_x, cup_y, cup_z;
}

void CustomController::JointTrajectoryCallback(const trajectory_msgs::JointTrajectoryPtr &msg)
{
    points = msg->points;
    data_collect_start_ = true;
    init_time = true;
    num_waypoint = points.size();
}

void CustomController::JointTargetCallback(const sensor_msgs::JointStatePtr &msg)
{
    t_0_ = (msg->header.stamp - init).toSec();
    q_0_ = rd_.q_.block<8,1>(25,0);
    qdot_0_ = rd_.q_dot_.block<8,1>(25,0);
    right_arm_target = msg->position;
    target_received = true;
}

void CustomController::JointReplayCallback(const sensor_msgs::JointStatePtr &msg)
{
    t_0_ = (msg->header.stamp - init).toSec();
    q_0_33 = rd_.q_;
    qdot_0_33 = rd_.q_dot_;
    right_arm_target = msg->position;
    target_received = true;
    last_received_time_ = ros::Time::now();
}

Eigen::Matrix3d CustomController::Quat2rotmatrix(double q0, double q1, double q2, double q3)
{
    double r00 = 2 * (q0 * q0 + q1 * q1) - 1 ;
    double r01 = 2 * (q1 * q2 - q0 * q3) ;
    double r02 = 2 * (q1 * q3 + q0 * q2) ;

    double r10 = 2 * (q1 * q2 + q0 * q3) ;
    double r11 = 2 * (q0 * q0 + q2 * q2) - 1 ;
    double r12 = 2 * (q2 * q3 - q0 * q1) ;

    double r20 = 2 * (q1 * q3 - q0 * q2) ;
    double r21 = 2 * (q2 * q3 + q0 * q1) ;
    double r22 = 2 * (q0 * q0 + q3 * q3) - 1 ;

    Eigen::Matrix3d rot_matrix;
    rot_matrix << r00, r01, r02,
                    r10, r11, r12,
                    r20, r21, r22;
    return rot_matrix;
}

void CustomController::computePlanner()
{
}

void CustomController::copyRobotData(RobotData &rd_l)
{
    std::memcpy(&rd_cc_, &rd_l, sizeof(RobotData));
}

Eigen::MatrixXd CustomController::LowPassFilter(const Eigen::MatrixXd &input, const Eigen::MatrixXd &prev_res, const double &sampling_freq, const double &cutoff_freq)
{
    double rc = 1. / (cutoff_freq * 2 * M_PI);
    double dt = 1. / sampling_freq;
    double a = dt / (rc + dt);
    return prev_res + a * (input - prev_res);
}