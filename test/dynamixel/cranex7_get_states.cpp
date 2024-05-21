/*
    cranex7の位置を取得するプログラム
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

#define CRANEX7_DEVICENAME  "/dev/ttyUSB0"


map<string, int> DXL_ID = {
    {"J1", 2},
    {"J2", 3},
    {"J3", 4},
    {"J4", 5},
    {"J5", 6},
    {"J6", 7},
    {"J7", 8},
    {"J8", 9}
};

int main() {
    DXLHandler dxlHandler(CRANEX7_DEVICENAME, 3000000);

    //サーボを追加
    for (auto& kv : DXL_ID) {
        dxlHandler.addServo(kv.second, DynamixelType::XM430);
    }

    dxlHandler.setup();
    //トルクをすべてオフに
    for (auto& kv : DXL_ID) {
        dxlHandler.setTorqueEnable(kv.second, false);
    }

    int dxl_comm_result = COMM_TX_FAIL;             // Communication result


    map<int, double> goal_currents;
    map<int, double> velocities;
    map<int, double> tau_d;
    map<int, double> tau_r;

    while (1) {
        // 現在の状態を取得
        auto currents = dxlHandler.getCurrents();
        auto positions = dxlHandler.getPositions();

        // 現在の状態を表示
        for (const auto& id : dxlHandler.dxl_ids) {
            printf("ID: %d, Cur: %4f mA, Vel: %4f, Pos: %4f, tau_d:%4f, tau_r:%4f \n", id, goal_currents[id], velocities[id], positions[id], tau_d[id], tau_r[id]);
        }

        // 一定時間待機[0.1s]
        usleep(10);
    }

    dxlHandler.shutdown();

    return 0;
}