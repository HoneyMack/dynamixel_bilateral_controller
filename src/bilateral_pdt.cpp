/*
    角度・角速度・トルクフィードバックcontrollerでバイラテ制御を行う(未完)
    xl330-m077を使用
*/

#include <fcntl.h>
#include <termios.h>
#define STDIN_FILENO 0


#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <list>
#include <vector>
#include <map>
#include <initializer_list>


#include <dynamixel_sdk/dynamixel_sdk.h> // Uses DYNAMIXEL SDK library
#include <unistd.h>


#include "dxl_handler.hpp"
#include "dxl_const.hpp"
#include "observers.hpp"


using namespace std;

// SERVO_ID
#define DXL_ID_LEADER 1
#define DXL_ID_FOLLOWER 2
#define DEVICENAME  "/dev/ttyUSB1"


const double KP = 1.0;
const double KD = 1e-6;
const double KT = 1e-2;


int getch() {
    struct termios oldt, newt;
    int ch;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
}



int main() {
    DXLHandler dxlHandler(DEVICENAME);
    dxlHandler.addServo(DXL_ID_LEADER);
    dxlHandler.addServo(DXL_ID_FOLLOWER);
    dxlHandler.setup();

    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    //モータのパラメータ
    double J=0.01038306503027827, D=0.06542060735147959;

    //疑似微分器
    double T_control = 0.002;
    double cutoff_diff = 20; //10Hz
    double cutoff_disturbance = 10; //10Hz
    double cutoff_reaction = 5; //10Hz

    FilterdDifferentiator Leader_filter(cutoff_diff, T_control);
    FilterdDifferentiator Follower_filter(cutoff_diff, T_control);
    //オブザーバ
    // 外乱・反力を計算
    DOB dob(J, D, cutoff_disturbance, T_control);
    RFOB rfob(J, D, cutoff_reaction, T_control);

    while (1) {
        // 現在の状態を取得
        auto currents = dxlHandler.getCurrents();
        auto positions = dxlHandler.getPositions();
        // auto velocities = dxlHandler.getVelocities();
        //角速度を疑似微分により求める
        double vel_leader = Leader_filter.filter(positions[DXL_ID_LEADER]);
        double vel_follower = Follower_filter.filter(positions[DXL_ID_FOLLOWER]);
        map<int, double> velocities = {
            {DXL_ID_LEADER, vel_leader},
            {DXL_ID_FOLLOWER, vel_follower}
        };

        // // 現在の状態を表示
        // for (const auto& id : dxlHandler.dxl_ids) {
        //     printf("ID: %d, Current: %f mA, Velocity: %f, Position: %f \n", id, currents[id], velocities[id], positions[id]);
        // }

        
        // 目標電流を設定
        double pos_f = positions[DXL_ID_FOLLOWER];
        double pos_l = positions[DXL_ID_LEADER];
        double vel_f = velocities[DXL_ID_FOLLOWER];
        double vel_l = velocities[DXL_ID_LEADER];
        //外乱・反力を計算
        double tau_d_l = dob.step(currents[DXL_ID_LEADER], vel_l);
        double tau_d_f = dob.step(currents[DXL_ID_FOLLOWER], vel_f);
        double tau_r_l = rfob.step(currents[DXL_ID_LEADER], vel_l, tau_d_l);
        double tau_r_f = rfob.step(currents[DXL_ID_FOLLOWER], vel_f, tau_d_f);



        double goal_current_l = KP * (pos_f - pos_l) + KD * (vel_f - vel_l);
        double goal_current_f = KP * (pos_l - pos_f) + KD * (vel_l - vel_f);

        //力フィードバックを追加
        goal_current_f += - KT *(tau_r_f + tau_d_f) + tau_d_l - tau_r_l;
        goal_current_l += - KT *(tau_r_l + tau_d_l) + tau_d_f - tau_r_f;
        // goal_current_f += - KT *(tau_r_f + tau_d_f) ;
        // goal_current_l += - KT *(tau_r_l + tau_d_l);

        map<int, double> goal_currents = {
            {DXL_ID_LEADER, goal_current_l},
            {DXL_ID_FOLLOWER, goal_current_f}
        };
        dxlHandler.setCurrents(goal_currents);
    }

    dxlHandler.shutdown();

    return 0;
}