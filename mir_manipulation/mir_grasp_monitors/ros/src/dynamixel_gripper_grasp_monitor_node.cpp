/*
 * dynamixel_gripper_grasp_monitor_node.cpp
 *
 *  Created on: Jul 11, 2015
 *      Author: Frederik Hegger
 */

#include <mir_grasp_monitors/dynamixel_gripper_grasp_monitor_node.h>

DynamixelGripperGraspMonitorNode::DynamixelGripperGraspMonitorNode() :
    joint_states_received_(false),
    event_in_received_(false),
    current_state_(INIT),
    loop_rate_init_state_(ros::Rate(100.0))
{
    ros::NodeHandle nh("~");

    nh.param("load_threshold", load_threshold_, 0.3);
    nh.param("position_error_threshold", position_error_threshold_, 0.1);

    pub_event_ = nh.advertise<std_msgs::String>("event_out", 1);
    sub_event_ = nh.subscribe("event_in", 10, &DynamixelGripperGraspMonitorNode::eventCallback, this);
    sub_dynamixel_motor_states_ = nh.subscribe("dynamixel_motor_states", 10, &DynamixelGripperGraspMonitorNode::jointStatesCallback, this);
    sub_object_name_ = nh.subscribe("object_name", 10, &DynamixelGripperGraspMonitorNode::objectNameCallback, this);

    // youbot_2: empty: 0.03
    //addThreshold("bearing", 0.12);
    //addThreshold("distance_tube", 0.12);
    //addThreshold("bearing_box", 0.15);
    //addThreshold("motor", 0.18);
    //addThreshold("s40_40_g", 0.15);
    //addThreshold("s40_40_b", 0.15);
    //addThreshold("m30", 0.15);
    //addThreshold("m20", 0.12);
    //addThreshold("r20", 0.12);
    //addThreshold("f20_20_g", 0.08);
    //addThreshold("f20_20_b", 0.08);
    
    // youbot 4: empty: 0.09
    addThreshold("bearing", 0.15);
    addThreshold("distance_tube", 0.15);
    addThreshold("bearing_box", 0.20);
    addThreshold("motor", 0.20);
    addThreshold("s40_40_g", 0.19);
    addThreshold("s40_40_b", 0.19);
    addThreshold("m30", 0.20);
    addThreshold("m20", 0.15);
    addThreshold("r20", 0.15);
    addThreshold("f20_20_g", 0.13);
    addThreshold("f20_20_b", 0.13);
;
}


DynamixelGripperGraspMonitorNode::~DynamixelGripperGraspMonitorNode()
{
    pub_event_.shutdown();
    sub_event_.shutdown();
    sub_dynamixel_motor_states_.shutdown();
}

void DynamixelGripperGraspMonitorNode::addThreshold(std::string str, float f) {
   object_threshold_map[str] = f;
   for(int i = 0;i<100;i++) {
       std::ostringstream stringStream1;
       stringStream1 << str << "-" << std::setfill('0') << std::setw(2) << i;
       std::string id1 = stringStream1.str();
       object_threshold_map[id1] = f;
       std::ostringstream stringStream2;
       stringStream2 << str << "_" << std::setfill('0') << std::setw(2) << i;
       std::string id2 = stringStream2.str();
       object_threshold_map[id2] = f;
   }
}

void DynamixelGripperGraspMonitorNode::jointStatesCallback(const dynamixel_msgs::JointState::Ptr &msg)
{
    joint_states_ = msg;
    joint_states_received_ = true;
}

void DynamixelGripperGraspMonitorNode::eventCallback(const std_msgs::String::ConstPtr &msg)
{
    event_in_ = *msg;
    event_in_received_ = true;
}

void DynamixelGripperGraspMonitorNode::objectNameCallback(const std_msgs::String::ConstPtr &msg){
    object_name_ = msg->data;
    std::transform(object_name_.begin(), object_name_.end(), object_name_.begin(), tolower);
    ROS_DEBUG("object_name: %s", object_name_.c_str());
}

void DynamixelGripperGraspMonitorNode::update()
{
    checkForNewEvent();

    switch (current_state_)
    {
    case INIT:
        init_state();
        break;
    case IDLE:
        idle_state();
        break;
    case RUN:
        run_state();
        break;
    }
}

void DynamixelGripperGraspMonitorNode::checkForNewEvent()
{
    if (!event_in_received_)
        return;

    ROS_INFO_STREAM("Received event: " << event_in_.data);

    if (event_in_.data == "e_trigger")
        current_state_ = IDLE;
    else
        ROS_ERROR_STREAM("Event not supported: " << event_in_.data);

    event_in_received_ = false;
}

void DynamixelGripperGraspMonitorNode::init_state()
{
    loop_rate_init_state_.sleep();
}

void DynamixelGripperGraspMonitorNode::idle_state()
{
    // wait for incoming data
    if (joint_states_received_)
        current_state_ = RUN;

    joint_states_received_ = false;
}

void DynamixelGripperGraspMonitorNode::run_state()
{
    std_msgs::String event_out;

    if (isObjectGrasped())
        event_out.data = "e_object_grasped";
    else
        event_out.data = "e_object_not_grasped";

    pub_event_.publish(event_out);

    current_state_ = INIT;
}

bool DynamixelGripperGraspMonitorNode::isObjectGrasped()
{
    ROS_DEBUG("load:     %f, threshold: %f", std::abs(joint_states_->load), load_threshold_);
    ROS_DEBUG("position error: %f, threshold: %f", std::abs(joint_states_->error), position_error_threshold_);
    if(object_threshold_map.find(object_name_) == object_threshold_map.end()){
      return true;
    }
    if (std::abs(joint_states_->load) >= object_threshold_map[object_name_]) {
        return true;
    }
    return false;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "grasp_monitor");
    ros::NodeHandle nh("~");

    DynamixelGripperGraspMonitorNode grasp_monitor;

    ros::Rate loop_rate(100);

    while (ros::ok())
    {
        ros::spinOnce();

        grasp_monitor.update();

        loop_rate.sleep();
    }


    return 0;
}
