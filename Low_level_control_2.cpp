/*
Author: Zhao-Shun Cheng, Yi-Cheng Chiu form NCKU Meclab
Email: e14051148@gs.ncku.edu.tw, n26104298@gs.ncku.edu.tw
Date: 2021/09
*/

#include <ros/ros.h>
#include <autoware_msgs/CanInfo.h>
#include <autoware_msgs/Velocity_info.h>
#include <autoware_msgs/LowLevelControl.h>
#include <time.h>
#include <iostream>

using namespace std;

autoware_msgs::LowLevelControl Param;
autoware_msgs::CanInfo vehicle;

bool load = false;
bool logic_throttle, logic_brake;

// ----- add by yc --------- //
//Audi A8 engine map
float A8_Throttle[10] = {0, 20, 30, 40, 50, 60, 70, 80, 90, 100};
float A8_EngineSpeed[12] = {800, 1200, 1400, 1600, 2000, 2400, 2800, 3200, 3600, 4000, 4400, 4800};
float A8_Torque[12][10] = {{-54, 97, 109, 121, 161, 201, 241, 282, 322, 362},
                        {-60, 114, 128, 142, 189, 236, 283, 331, 378, 425},
                        {-63, 120, 135, 150, 200, 250, 300, 350, 400, 450},
                        {-66, 120, 135, 150, 200, 250, 300, 350, 400, 450},
                        {-72, 120, 135, 150, 200, 250, 300, 350, 400, 450},
                        {-77, 50, 100, 150, 200, 250, 300, 350, 400, 450},
                        {-83, 50, 100, 150, 200, 250, 300, 350, 400, 450},
                        {-88, 50, 100, 150, 200, 250, 300, 350, 400, 450},
                        {-95, 49, 98, 147, 196, 244, 293, 342, 391, 440},
                        {-100, 45, 90, 135, 180, 226, 271, 316, 361, 406},
                        {-106, 39, 78, 118, 157, 196, 235, 275, 314, 353},
                        {-111, 31, 62, 92, 123, 154, 185, 215, 246, 277}};
// ----- add by yc --------- //

float InverseEngineMap(float des_Torque, float current_rpm);
void logic();
void PedalControl();

void ControlCallback(const autoware_msgs::LowLevelControl& msg)
{
    Param = msg;
    load = true;
}

int main(int argc, char **argv)
{
    ros::init(argc ,argv, "Low_Level_Control_2");
    ros::NodeHandle nh;
    ros::Publisher pub;
    pub = nh.advertise<autoware_msgs::CanInfo>("can_info2",1000);

    ros::Subscriber sub;
    sub = nh.subscribe("/Prescan/Dynamic_Param_2", 1, ControlCallback);

    ros::Rate rate(10);

    while(ros::ok())
    {
        ros::spinOnce();
        if(!load)
            ROS_INFO("Without update Dynamic_Param!");
        else
        {
            cout << "------------ Start pedalControl Control -------------" << endl;
            logic();
            PedalControl();
            pub.publish(vehicle);
            cout << "------------ Finish pedalControl Control -------------" << endl << endl;
            // load = false;
            rate.sleep();
        }
        
    }
    return 0;
}

void logic()
{
    float min_T[] = {-54, -60, -63, -66, -72, -77, -83, -88, -95, -100, -106, -111};
    float RPM = Param.engine_rpm;
    float T;

    // calculate torque when the throttle is 0% 
    if(RPM < 800)
        T = min_T[0];
    else if(RPM > 4800)
        T = min_T[11];
    else
    {
        for(int i = 1; i < sizeof(min_T) / sizeof(min_T[0]); i++)
        {
            if(RPM <= A8_EngineSpeed[i] && RPM >= A8_EngineSpeed[i-1])
            {
                T = min_T[i-1] + (min_T[i] - min_T[i-1]) * (RPM - A8_EngineSpeed[i-1]) / (A8_EngineSpeed[i] - A8_EngineSpeed[i-1]);
                break;
            }
        }
    }

    float acc_res = T * Param.ratio_final_driver * Param.gear_ratio / (Param.wheel_radius * Param.mass) - Param.exteral_force / Param.mass;
    cout << "acc_res : " << acc_res << ", desired_acc : " << Param.desired_acc << endl;
    if(Param.desired_acc >= acc_res + 0.1)
    {
        logic_throttle = true;
        logic_brake = false;
    }
    else if(Param.desired_acc <= acc_res - 0.1)
    {
        logic_throttle = false;
        logic_brake = true;
    }
    else
    {
        logic_throttle = false;
        logic_brake = false;
    }
}

void PedalControl()
{   
    if(logic_throttle)
    {
        cout << "Acceleration mode !!!" << endl;
        float T = (Param.exteral_force + Param.mass * Param.desired_acc) * Param.wheel_radius / (Param.gear_ratio * Param.ratio_final_driver);
        cout << "desired torque :" << T << ", Engine RPM : " << Param.engine_rpm << endl; // debug
        
        float desired_throttle = InverseEngineMap(T, Param.engine_rpm);
        if(desired_throttle > 100)
            desired_throttle = 100;
        if(desired_throttle < 0)  
            desired_throttle = 0;   
        vehicle.throttle = desired_throttle;
        vehicle.brake = 0;
    }
    else if(logic_brake)
    {
        cout << "Deceleration mode !!!" << endl;
        float T = (-Param.wheel_radius) * (Param.mass * Param.desired_acc + Param.exteral_force) + Param.wheel_torque;
        float bar = T / Param.q_factor;
        vehicle.throttle = 0;
        vehicle.brake = bar;
    }
    else
    {
        cout << "Don't do anything !!!" << endl;
        vehicle.throttle = 0;
        vehicle.brake = 0;
    }
    logic_throttle = false;
    logic_brake = false;
}

int findDifference(float input, float *compare, int length, int type){
for(int i = 0;i < length;i++){
    if(type == 0){// 0 for speed
        if(input < compare[i])
            return i - 1;
    }else if(type == 1){// 1 for torque
        if(input >= compare[9])
            return 9;
        if(input < compare[i])
            return i - 1;
        }
    }
}

float findScale(float input, float *compare, int row){
    return (input - compare[row]) / (compare[row + 1] - compare[row]);
}

float InverseEngineMap(float des_Torque, float current_rpm)
{
    float A8_TorqueDiff[10]={};
    float A8_ThrottleDiff = 0;
    int rpm_row, torque_row;
    float rpm_scale, torque_scale;

  

    if(current_rpm > A8_EngineSpeed[0] && current_rpm < A8_EngineSpeed[11] ){

        rpm_row = findDifference(current_rpm, A8_EngineSpeed, sizeof(A8_EngineSpeed) / sizeof(A8_EngineSpeed[0]), 0);
        rpm_scale = findScale(current_rpm, A8_EngineSpeed, rpm_row);

        for(int i = 0;i < 10;i++){// torque_diff
            A8_TorqueDiff[i] = (A8_Torque[rpm_row+1][i]-A8_Torque[rpm_row][i]) * rpm_scale + A8_Torque[rpm_row][i];
            ROS_INFO("A8_TorqueDiff:%f", A8_TorqueDiff[i]);
        }
    }else{
        if(current_rpm <= A8_EngineSpeed[0]){// rpm under 800 
            for(int i = 0;i < sizeof(A8_Throttle)/sizeof(A8_Throttle[0]);i++){
                A8_TorqueDiff[i] = A8_Torque[0][i];
                ROS_INFO("A8_TorqueDiff:%f", A8_TorqueDiff[i]);
            } 
        }
        if(current_rpm >= A8_EngineSpeed[11]){// rpm above 4800
            for(int i = 0;i < sizeof(A8_Throttle)/sizeof(A8_Throttle[0]);i++){
                A8_TorqueDiff[i] = A8_Torque[11][i];
                ROS_INFO("A8_TorqueDiff:%f", A8_TorqueDiff[i]);
            } 
        }
    }
    torque_row = findDifference(des_Torque, A8_TorqueDiff, sizeof(A8_TorqueDiff)/sizeof(A8_TorqueDiff[0]), 1);
    if(torque_row == 9){
        A8_ThrottleDiff = 100;
    }else{
        torque_scale= findScale(des_Torque, A8_TorqueDiff, torque_row);
        A8_ThrottleDiff = (A8_Throttle[torque_row+1]-A8_Throttle[torque_row]) * torque_scale + A8_Throttle[torque_row];
    }
    ROS_INFO("rpm_row:%d, rpm_scale:%f, torque_row:%d, torque_scale:%f" , rpm_row, rpm_scale, torque_row, torque_scale); 
    ROS_INFO("A8_ThrottleDiff:%f", A8_ThrottleDiff);
    return A8_ThrottleDiff;
}