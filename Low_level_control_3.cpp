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
//Toyota Previa MPV
float MPV_Throttle[10] = {0, 20, 30, 40, 50, 60, 70, 80, 90, 100};
float MPV_EngineSpeed[14] = {600, 800, 1000, 1500, 2000, 2500, 3000, 3500, 4000, 4500, 5000, 5600, 6000, 6500};
float MPV_Torque[14][10] = {{-40, 32, 36, 40, 53, 67, 80, 93, 107, 120},
                        {-43, 40, 45, 50, 67, 83, 100, 117, 133, 150},
                        {-47, 46, 52, 58, 78, 97, 117, 136, 156, 175},
                        {-53, 50, 57, 63, 84, 106, 127, 148, 169, 190},
                        {-57, 54, 60, 67, 89, 111, 133, 156, 178, 200},
                        {-60, 22, 45, 67, 90, 112, 135, 157, 180, 202},
                        {-63, 23, 46, 68, 91, 114, 137, 159, 182, 205},
                        {-70, 23, 47, 70, 93, 117, 140, 163, 187, 210},
                        {-73, 25, 50, 75, 100, 125, 150, 175, 200, 225},
                        {-77, 25, 50, 75, 100, 125, 150, 175, 200, 225},
                        {-80, 24, 48, 72, 96, 119, 143, 167, 191, 215},
                        {-83, 22, 44, 65, 87, 109, 131, 152, 174, 196},
                        {-90, 20, 40, 60, 80, 100, 120, 140, 160, 180},
                        {-100, 18, 36, 53, 71, 89, 107, 124, 142, 160}};
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
    ros::init(argc ,argv, "Low_Level_Control_3");
    ros::NodeHandle nh;
    ros::Publisher pub;
    pub = nh.advertise<autoware_msgs::CanInfo>("can_info3",1000);

    ros::Subscriber sub;
    sub = nh.subscribe("/Prescan/Dynamic_Param_3", 1, ControlCallback);

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
            if(RPM <= MPV_EngineSpeed[i] && RPM >= MPV_EngineSpeed[i-1])
            {
                T = min_T[i-1] + (min_T[i] - min_T[i-1]) * (RPM - MPV_EngineSpeed[i-1]) / (MPV_EngineSpeed[i] - MPV_EngineSpeed[i-1]);
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
    float MPV_TorqueDiff[10]={};
    float MPV_ThrottleDiff = 0;
    int rpm_row, torque_row;
    float rpm_scale, torque_scale;

  

    if(current_rpm > MPV_EngineSpeed[0] && current_rpm < MPV_EngineSpeed[13] ){

        rpm_row = findDifference(current_rpm, MPV_EngineSpeed, sizeof(MPV_EngineSpeed) / sizeof(MPV_EngineSpeed[0]), 0);
        rpm_scale = findScale(current_rpm, MPV_EngineSpeed, rpm_row);

        for(int i = 0;i < 10;i++){// torque_diff
            MPV_TorqueDiff[i] = (MPV_Torque[rpm_row+1][i]-MPV_Torque[rpm_row][i]) * rpm_scale + MPV_Torque[rpm_row][i];
            ROS_INFO("MPV_TorqueDiff:%f", MPV_TorqueDiff[i]);
        }
    }else{
        if(current_rpm <= MPV_EngineSpeed[0]){// rpm under 800 
            for(int i = 0;i < sizeof(MPV_Throttle)/sizeof(MPV_Throttle[0]);i++){
                MPV_TorqueDiff[i] = MPV_Torque[0][i];
                ROS_INFO("MPV_TorqueDiff:%f", MPV_TorqueDiff[i]);
            } 
        }
        if(current_rpm >= MPV_EngineSpeed[13]){// rpm above 4800
            for(int i = 0;i < sizeof(MPV_Throttle)/sizeof(MPV_Throttle[0]);i++){
                MPV_TorqueDiff[i] = MPV_Torque[13][i];
                ROS_INFO("MPV_TorqueDiff:%f", MPV_TorqueDiff[i]);
            } 
        }
    }
    torque_row = findDifference(des_Torque, MPV_TorqueDiff, sizeof(MPV_TorqueDiff)/sizeof(MPV_TorqueDiff[0]), 1);
    if(torque_row == 9){
        MPV_ThrottleDiff = 100;
    }else{
        torque_scale= findScale(des_Torque, MPV_TorqueDiff, torque_row);
        MPV_ThrottleDiff = (MPV_Throttle[torque_row+1]-MPV_Throttle[torque_row]) * torque_scale + MPV_Throttle[torque_row];
    }
    ROS_INFO("rpm_row:%d, rpm_scale:%f, torque_row:%d, torque_scale:%f" , rpm_row, rpm_scale, torque_row, torque_scale); 
    ROS_INFO("MPV_ThrottleDiff:%f", MPV_ThrottleDiff);
    return MPV_ThrottleDiff;
}