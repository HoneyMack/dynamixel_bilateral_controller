/*
    indentify_system.cpp
    システム同定のために必要な入出力データを取得するプログラム
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


using namespace std;



// SERVO_ID
#define DXL_ID 1
#define DEVICENAME  "/dev/ttyUSB0"


const double KP = 1.0;
const double KD = 1e-7;


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
    dxlHandler.addServo(DXL_ID);
    dxlHandler.setup();
    this_thread::sleep_for(chrono::seconds(1)); // 1s

    int dxl_comm_result = COMM_TX_FAIL;             // Communication result

    map<int, double> currents, positions, velocities;

    atomic<bool> isThreadFinished(false);

    // 入力値の設定    
    // vector<double> cur_refs = { 50.0, 60.0, 70.0, 80.0, 90.0, 100.0 };
    int num_samples = 3000;
    vector<double> cur_refs = { 70.0, 100.0, 150, 200};
    // 出力データの保存先
    vector<vector<tuple<double, double, double, double>>> data; // 時刻[s]，電流[ms]，位置[deg]，速度[rpm]


    function<void(int)> controller = [&](int num_samples) {
        for (int cur_idx = 0; cur_idx < cur_refs.size(); cur_idx++) {
            // dataの要素を追加
            data.push_back(vector<tuple<double, double, double, double>>());

            // トルクをオフにして3s待つ
            dxlHandler.setCurrents({ {DXL_ID, 0.0} });
            this_thread::sleep_for(chrono::seconds(3));

            // 現在の状態を取得
            currents = dxlHandler.getCurrents();
            positions = dxlHandler.getPositions();
            //velocities = dxlHandler.getVelocities(); //NOTE: 500 Hz出すために角速度は取得しない
            auto start = chrono::system_clock::now();
            data[cur_idx].push_back(make_tuple(0.0, currents[DXL_ID], positions[DXL_ID], velocities[DXL_ID]));

            // トルクをオンにする
            dxlHandler.setCurrents({ {DXL_ID, cur_refs[cur_idx]} });
            for (int i = 0; i < num_samples; i++) {
                // 現在の状態を取得
                currents = dxlHandler.getCurrents();
                positions = dxlHandler.getPositions();
                //velocities = dxlHandler.getVelocities(); //NOTE: 500 Hz出すために角速度は取得しない
                auto end = chrono::system_clock::now();
                chrono::duration<double>  e_time = end - start;
                data[cur_idx].push_back(make_tuple(e_time.count(), currents[DXL_ID], positions[DXL_ID], velocities[DXL_ID]));
            }
        }

        // トルクをオフにして3s待つ
        dxlHandler.setCurrents({ {DXL_ID, 0.0} });
        this_thread::sleep_for(chrono::seconds(3));
        // 終了フラグを立てる
        isThreadFinished = true;
        };

    thread controller_thread(controller, num_samples);
    // controller_thread.detach();

    while (1) {
        // 現在の状態を表示
        for (const auto& id : dxlHandler.dxl_ids) {
            printf("ID: %d, Current: %f mA, Velocity: %f, Position: %f \n", id, currents[id], velocities[id], positions[id]);
        }

        this_thread::sleep_for(chrono::milliseconds(500)); // 0.5 ms
        if (isThreadFinished)
            break;
    }


    controller_thread.join();

    dxlHandler.shutdown();

    // データをcsv形式で保存
    for (int i = 0; i < data.size(); i++) {
        FILE* fp;
        string filename = "data" + to_string(i) + ".csv";
        fp=fopen(filename.c_str(), "w");
        fprintf(fp, "time, current, position, velocity\n");
        for (const auto& d : data[i]) {
            fprintf(fp, "%f, %f, %f, %f\n", get<0>(d), get<1>(d), get<2>(d), get<3>(d));
        }
        fclose(fp);
        
    }

    return 0;
}