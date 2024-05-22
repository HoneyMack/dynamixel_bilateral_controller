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
#include <algorithm>

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

map<int, double> convert_unit_rad_to_deg(map<int, double> param_t) {
    //単位系が違うので変換
    map<int, double> param_c;
    for (auto kv : param_t) {
        param_c[kv.first] = kv.second * 3.14159 / 180.0;
    }
    return param_c;
}

bool clip_backlash(double &val, double backlash) {
    //バックラッシュ分はクリップ．クリップしたらtrueを返す
    if (val > backlash) {
        val -= backlash;
        return true;
    }
    else if (val < -backlash) {
        val += backlash;
        return true;
    }
    else {
        val = 0;
        return false;
    }
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
        else {
            leaderDxlHandler.addServo(kv.first, DynamixelType::XM430);
            followerDxlHandler.addServo(kv.first, DynamixelType::XM430);
        }
    }

    leaderDxlHandler.setup(false, 0);
    followerDxlHandler.setup(false, 0);

    //J3は位置制御で180度でロック
    leaderDxlHandler.setOperationMode(4, 3);
    followerDxlHandler.setOperationMode(4, 3);

    //トルクオン
    for (auto kv : DXL_TO_JOINT_ID) {
        leaderDxlHandler.setTorqueEnable(kv.first, true);
        followerDxlHandler.setTorqueEnable(kv.first, true);
    }

    //J3は位置制御で180度でロック
    leaderDxlHandler.setPosition(4, 180);
    followerDxlHandler.setPosition(4, 180);

    //疑似微分器
    // double T_control = 1.0 / 500;
    double T_control = 1.0 / 500;
    double cutoff_diff = 20;
    double cutoff_disturbance = 0.5;
    double cutoff_reaction = 0.5;

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
        {8, 0.007},
    };
    map<int, double> Ds = {
        {1, 0.0501},
        {2, 0.0000},
        {3, 0.242},
        {4, 0.000},
        {5, 0.040},
        {6, 0.0391},
        {7, 0.0500},
        {8, 0.0210}
    };
    map<int, double> Ms_lead = {
        {1, 2.094 / 4},
        {2, 1.151 / 3},
        {3, 1.183 / 3},

    };
    map<int, double> Ms_follow = {
        {1, 2.294 / 4},
        {2, 1.451 / 3},
        {3, 1.483 / 3},
    };
    // //重力補償なし
    // map<int, double> Ms_lead;
    // map<int, double> Ms_follow;

    map<int, double> Kps = {
        {1, 256.0},
        {2, 196.0},
        {3, 961.0},
        {4, 144.0},
        {5, 289.0},
        {6, 324.0},
        {7, 144.0},
        {8, 324.0*4}
    };

    map<int, double> Kds = {
        {1, 40.0},
        {2, 28.0},
        // {3, 66.0},
        {4, 24.0},
        // {5, 34.0/2},
        // {6, 36.0/2},
        // {7, 24.0},
        // {8, 36.0/2}
    };

    //Kds 0
    // map<int, double> Kds;

    // map<int, double> Kts = {
    //     {1, 0.70},
    //     {2, 0.70},
    //     {3, 1.00},
    //     {4, 1.00},
    //     {5, 0.80},
    //     {6, 1.00},
    //     {7, 0.80},
    //     {8, 1.00},
    // };
    map<int, double> Kts = {
        {1, 1.00},
        {2, 1.00},
        //     {3, 1.00},
            {4, 0.90},
            //{5, 0.80}, //ここをオンにするとバイラテ制御がうまくいかない
            //{6, 0.80},
            //{7, 0.80},
            {8, 1.0},
    };

    //角度の単位変換に伴うパラメータの変換
    Js = convert_unit_rad_to_deg(Js);
    Ds = convert_unit_rad_to_deg(Ds);


    // Kps = convert_unit_rad_to_deg(Kps);
    // Kds = convert_unit_rad_to_deg(Kds);



    // Kps -> Js*Kps, Kds -> Js*Kds


    //オブザーバ:外乱・反力を計算
    Cranex7Observer cranex_obs_l(Js, Ds, Ms_lead, cutoff_disturbance, T_control);
    Cranex7Observer cranex_obs_f(Js, Ds, Ms_follow, cutoff_disturbance, T_control);

    map<int, double> torque_goal_l, torque_goal_f;

    atomic<bool> finish_flag(false), start_get_l_state_flag(false), start_get_f_state_flag(false);
    map<int, double> vel_l, vel_f, pos_l, pos_f;
    //実行時間計測
    const int mean_data_num = 100;
    chrono::system_clock::time_point time_start, time_end;
    //分散計算用
    vector<double> elapsed_times(mean_data_num, 0.0);

    time_start = chrono::system_clock::now();
    function<void()> get_l_state = [&]() {
        while (finish_flag == false) {
            if (start_get_l_state_flag == true) {
                vel_l = convert_dxl_to_joint_idx(leaderDxlHandler.getVelocities());
                pos_l = convert_dxl_to_joint_idx(leaderDxlHandler.getPositions());

                for (auto& kv : vel_l) {
                    kv.second *= 6;//rpm -> deg/s
                }

                start_get_l_state_flag = false;
            }
            // 0.1ms待つ
            this_thread::sleep_for(chrono::microseconds(100));
        }
        };

    function<void()> get_f_state = [&]() {
        while (finish_flag == false) {
            if (start_get_f_state_flag == true) {
                vel_f = convert_dxl_to_joint_idx(followerDxlHandler.getVelocities());
                pos_f = convert_dxl_to_joint_idx(followerDxlHandler.getPositions());

                for (auto& kv : vel_f) {
                    kv.second *= 6;//rpm -> deg/s
                }

                start_get_f_state_flag = false;
            }
            // 0.1ms待つ
            this_thread::sleep_for(chrono::microseconds(100));
        }
        };


    function<void()> controller = [&]() {
        int counter = 0;
        while (finish_flag == false) {
            //情報の取得開始
            start_get_l_state_flag = true;
            start_get_f_state_flag = true;

            while (finish_flag == false &&
                (start_get_l_state_flag == true || start_get_f_state_flag == true)) {
                this_thread::sleep_for(chrono::microseconds(100));
            }

            //外乱・反力を計算
            auto tau_d_l = cranex_obs_l.step_torque_disturb(torque_goal_l, pos_l, vel_l);
            auto tau_d_f = cranex_obs_f.step_torque_disturb(torque_goal_f, pos_f, vel_f);
            auto tau_r_l = cranex_obs_l.step_torque_react(torque_goal_l, pos_l, vel_l, tau_d_l);
            auto tau_r_f = cranex_obs_f.step_torque_react(torque_goal_f, pos_f, vel_f, tau_d_f);


            //目標電流を計算
            for (auto& kv : pos_l) {
                const double backlash = 0.25 * 2 + 0.1;
                double d_pos_fl = pos_f[kv.first] - pos_l[kv.first];
                double d_vel_fl = vel_f[kv.first] - vel_l[kv.first];
                bool cliped = clip_backlash(d_pos_fl, backlash);
                if (cliped){
                    // クリップされていたら角速度も0
                    d_vel_fl = 0;
                }

                torque_goal_l[kv.first] = Js[kv.first] / 2 * Kps[kv.first] * d_pos_fl + Js[kv.first] / 2 * Kds[kv.first] * d_vel_fl;
                torque_goal_f[kv.first] = Js[kv.first] / 2 * Kps[kv.first] * -d_pos_fl + Js[kv.first] / 2 * Kds[kv.first] * -d_vel_fl;

                // torque_goal_l[kv.first] = Js[kv.first] / 2 * Kps[kv.first] * (pos_f[kv.first] - pos_l[kv.first]) + Js[kv.first] / 2 * Kds[kv.first] * (vel_f[kv.first] - vel_l[kv.first]);
                // torque_goal_f[kv.first] = Js[kv.first] / 2 * Kps[kv.first] * (pos_l[kv.first] - pos_f[kv.first]) + Js[kv.first] / 2 * Kds[kv.first] * (vel_l[kv.first] - vel_f[kv.first]);
            }

            // 力フィードバックを追加
            for (auto& kv : tau_r_l) {
                torque_goal_l[kv.first] += -Kts[kv.first] / 2 * (tau_r_f[kv.first] + tau_r_l[kv.first]) + tau_d_l[kv.first] - tau_r_l[kv.first];
                torque_goal_f[kv.first] += -Kts[kv.first] / 2 * (tau_r_l[kv.first] + tau_r_f[kv.first]) + tau_d_f[kv.first] - tau_r_f[kv.first];
            }

            //インデックス調整
            torque_goal_l = convert_joint_to_dxl_idx(torque_goal_l);
            torque_goal_f = convert_joint_to_dxl_idx(torque_goal_f);

            // トルクが入力範囲内に収まるようにclip
            torque_goal_l = leaderDxlHandler.clipTorques(torque_goal_l);
            torque_goal_f = followerDxlHandler.clipTorques(torque_goal_f);

            leaderDxlHandler.setTorques(torque_goal_l);
            followerDxlHandler.setTorques(torque_goal_f);

            time_end = chrono::system_clock::now();
            double elapsed = chrono::duration_cast<chrono::milliseconds>(time_end - time_start).count();
            elapsed_times[counter] = elapsed;
            counter++;

            if (counter >= mean_data_num) {
                // 周波数・制御周期を表示
                double interval_average = 0.0, interval_std = 0.0;
                for (const auto& interval : elapsed_times) {
                    interval_average += interval;
                    interval_std += interval * interval;
                }
                interval_average /= mean_data_num;
                interval_std = sqrt(interval_std / mean_data_num - interval_average * interval_average);
                printf("interval: %5.3f +- %5.3f [ms]\n", interval_average, interval_std);
                printf("freq: %5.3f +- %5.3f\n", 1000.0 / interval_average, 1000.0 * interval_std/(interval_average*interval_average));
                // 現在の状態を表示
                //torque
                printf("torque:\t");
                for (const auto& id : leaderDxlHandler.dxl_ids) {
                    printf("%6.3f,", torque_goal_l[id]);
                }
                printf("\n");

                //tau_d
                printf("tau_d:\t");
                for (const auto& id : leaderDxlHandler.dxl_ids) {
                    printf("%6.3f,", tau_d_l[id]);
                }
                printf("\n");

                //tau_r
                printf("tau_r:\t");
                for (const auto& id : leaderDxlHandler.dxl_ids) {
                    printf("%6.3f,", tau_r_l[id]);
                }
                printf("\n\n\n");
                counter = 0;
            }
            // // 現在の状態を表示
            // for (const auto& id : dxlHandler.dxl_ids) {
            //     printf("ID: %d, Cur: %4f mA, Vel: %4f, Pos: %4f, tau_d:%4f, tau_r:%4f \n", id, goal_currents[id], velocities[id], positions[id], tau_d[id], tau_r[id]);
            // }
            time_start = chrono::system_clock::now();
        }
        };
    thread get_l_state_thread(get_l_state), get_f_state_thread(get_f_state);
    thread controller_thread(controller);

    cout << "Press any key to finish" << endl;
    getch();
    finish_flag = true;
    controller_thread.join();
    get_l_state_thread.join();
    get_f_state_thread.join();


    leaderDxlHandler.shutdown();
    followerDxlHandler.shutdown();

    return 0;
}