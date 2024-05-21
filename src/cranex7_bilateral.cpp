/*
    角度・角速度・トルクフィードバックcontrollerでバイラテ制御を行う
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
#include <thread>
#include <atomic>
#include <chrono>
#include <functional>

#include <dynamixel_sdk/dynamixel_sdk.h> // Uses DYNAMIXEL SDK library
#include <unistd.h>


#include "dxl_handler.hpp"
#include "dxl_const.hpp"
#include "observers.hpp"
#include "cranex7_observer.hpp"


using namespace std;

// SERVO_ID
#define DXL_ID_LEADER 2
#define DXL_ID_FOLLOWER 2

#define LEADER_DEVICENAME  "/dev/ttyUSB0"
#define FOLLOWER_DEVICENAME  "/dev/ttyUSB1"


static map<int, int> DXL_TO_JOINT_ID = {
    {2, 1},
    {3, 2},
    {4, 3},
    {5, 4},
    {6, 5},
    {7, 6},
    {8, 7},
    {9, 8}
};

static map<int, int> JOINT_TO_DXL_ID = {
    {1, 2},
    {2, 3},
    {3, 4},
    {4, 5},
    {5, 6},
    {6, 7},
    {7, 8},
    {8, 9}
};



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

map<int, double> convert_dxl_to_joint_idx(map<int, double> val_dxl) {
    map<int, double> val_joint;
    for (auto kv : val_dxl) {
        val_joint[DXL_TO_JOINT_ID[kv.first]] = kv.second;
    }
    return val_joint;
}
map<int, double> convert_joint_to_dxl_idx(map<int, double> val_joint) {
    map<int, double> val_dxl;
    for (auto kv : val_joint) {
        val_dxl[JOINT_TO_DXL_ID[kv.first]] = kv.second;
    }
    return val_dxl;
}

int main() {
    DXLHandler leaderDxlHandler(LEADER_DEVICENAME, 3000000);
    DXLHandler followerDxlHandler(FOLLOWER_DEVICENAME, 3000000);
    for (auto kv : DXL_TO_JOINT_ID) {
        // J2はXM540, それ以外は XM430
        if (kv.second == 2) {
            leaderDxlHandler.addServo(kv.first, DynamixelType::XM540);
            followerDxlHandler.addServo(kv.first, DynamixelType::XM540);
        }
        else{
            leaderDxlHandler.addServo(kv.first, DynamixelType::XM430);
            followerDxlHandler.addServo(kv.first, DynamixelType::XM430);
        }
    }

    leaderDxlHandler.setup(true, 0);
    followerDxlHandler.setup(true, 0);

    //疑似微分器
    // double T_control = 1.0 / 500;
    double T_control = 1.0 / 250;
    double cutoff_diff = 20; 
    double cutoff_disturbance = 2.5;
    double cutoff_reaction = 2.5;

    //cranex7 observer のパラメータ
    // TODO: 要パラメータ調整
    map<int, double> Js = {
        {1, 0.012},
        {2, 0.113},
        {3, 0.012},
        {4, 0.040},
        {5, 0.006},
        {6, 0.007},
        {7, 0.006},
        {8, 0.007}
    };
    map<int, double> Ds = {
        {1, 0.050},
        {2, 0.0000},
        {3, 0.242},
        {4, 0.000},
        {5, 0.040},
        {6, 0.039},
        {7, 0.050},
        {8, 0.021}
    };
    map<int, double> Ms_lead = {
        {1, 2.094},
        {2, 1.151},
        {3, 1.183},

    };
    map<int, double> Ms_follow = {
        {1, 2.294},
        {2, 1.451},
        {3, 1.483},
    };

    map<int, double> Kps = {
        {1, 50.0},
        {2, 50.0},
        {3, 50.0},
        {4, 50.0},
        {5, 50.0},
        {6, 50.0},
        {7, 50.0},
        {8, 50.0}
    };

    map<int, double> Kds = {
        {1, 1e-3},
        {2, 1e-3},
        {3, 1e-3},
        {4, 1e-3},
        {5, 1e-3},
        {6, 1e-3},
        {7, 1e-3},
        {8, 1e-3}
    };

    map<int, double> Kts = {
        {1, 0.5},
        {2, 0.5},
        {3, 0.5},
        {4, 0.5},
        {5, 0.5},
        {6, 0.5},
        {7, 0.5},
        {8, 0.5},
    };

    //オブザーバ:外乱・反力を計算
    Cranex7Observer cranex_obs_l(Js, Ds, Ms_lead, cutoff_disturbance, T_control);
    Cranex7Observer cranex_obs_f(Js, Ds, Ms_follow, cutoff_disturbance, T_control);

    map<int, double> cur_goal_l, cur_goal_f;

    atomic<bool> finish_flag(false);
    function<void()> controller = [&]() {
        while (finish_flag == false) {
            // 現在の状態を取得
            // auto currents = dxlHandler.getCurrents();
            auto pos_l = convert_dxl_to_joint_idx(leaderDxlHandler.getPositions());
            auto vel_l = convert_dxl_to_joint_idx(leaderDxlHandler.getVelocities());
            auto pos_f = convert_dxl_to_joint_idx(followerDxlHandler.getPositions());
            auto vel_f = convert_dxl_to_joint_idx(followerDxlHandler.getVelocities());

            //rpm -> deg/s
            for (auto& kv : vel_l) {
                kv.second *= 6;
            }
            for (auto& kv : vel_f) {
                kv.second *= 6;
            }

            // // 角速度を疑似微分により求める場合
            // double vel_leader = Leader_filter.filter(positions[DXL_ID_LEADER]);
            // double vel_follower = Follower_filter.filter(positions[DXL_ID_FOLLOWER]);
            // map<int, double> velocities = {
            //     {DXL_ID_LEADER, vel_leader},
            //     {DXL_ID_FOLLOWER, vel_follower}
            // };

            // map<int, double> velocities = {
            //     {DXL_ID_LEADER, 0},
            //     {DXL_ID_FOLLOWER, 0}
            // };


            // 目標電流を設定

            //外乱・反力を計算
            auto tau_d_l = cranex_obs_l.step_torque_disturb(cur_goal_l, pos_l, vel_l);
            auto tau_d_f = cranex_obs_f.step_torque_disturb(cur_goal_f, pos_f, vel_f);
            auto tau_r_l = cranex_obs_l.step_torque_react(cur_goal_l, pos_l, vel_l, tau_d_l);
            auto tau_r_f = cranex_obs_f.step_torque_react(cur_goal_f, pos_f, vel_f, tau_d_f);


            //目標電流を計算
            for (auto& kv : pos_l) {
                cur_goal_l[kv.first] = Kps[kv.first] * (pos_f[kv.first] - pos_l[kv.first]) + Kds[kv.first] * (vel_f[kv.first] - vel_l[kv.first]);
                cur_goal_f[kv.first] = Kps[kv.first] * (pos_l[kv.first] - pos_f[kv.first]) + Kds[kv.first] * (vel_l[kv.first] - vel_f[kv.first]);
            }

            // 力フィードバックを追加
            for (auto& kv : tau_r_l) {
                cur_goal_l[kv.first] += -Kts[kv.first] * (tau_r_f[kv.first] + tau_r_l[kv.first]) + tau_d_l[kv.first] - tau_r_l[kv.first];
                cur_goal_f[kv.first] += -Kts[kv.first] * (tau_r_l[kv.first] + tau_r_f[kv.first]) + tau_d_f[kv.first] - tau_r_f[kv.first];
            }

            //インデックス調整
            cur_goal_l = convert_joint_to_dxl_idx(cur_goal_l);
            cur_goal_f = convert_joint_to_dxl_idx(cur_goal_f);

            leaderDxlHandler.setCurrents(cur_goal_l);
            followerDxlHandler.setCurrents(cur_goal_f);

            // // 現在の状態を表示
            // for (const auto& id : dxlHandler.dxl_ids) {
            //     printf("ID: %d, Cur: %4f mA, Vel: %4f, Pos: %4f, tau_d:%4f, tau_r:%4f \n", id, goal_currents[id], velocities[id], positions[id], tau_d[id], tau_r[id]);
            // }
        }
        };
    thread controller_thread(controller);

    cout << "Press any key to finish" << endl;
    getch();
    finish_flag = true;
    controller_thread.join();

    leaderDxlHandler.shutdown();
    followerDxlHandler.shutdown();

    return 0;
}