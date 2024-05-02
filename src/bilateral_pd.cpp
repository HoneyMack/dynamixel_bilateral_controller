/*
    PD controllerでバイラテ制御を行う
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


using namespace std;



// SERVO_ID
#define DXL_ID_LEADER 1
#define DXL_ID_FOLLOWER 2
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
    dxlHandler.addServo(DXL_ID_LEADER);
    dxlHandler.addServo(DXL_ID_FOLLOWER);
    dxlHandler.setup();

    int dxl_comm_result = COMM_TX_FAIL;             // Communication result

    while (1) {
        // 現在の状態を取得
        auto currents = dxlHandler.getCurrents();
        auto positions = dxlHandler.getPositions();
        auto velocities = dxlHandler.getVelocities();

        // 現在の状態を表示
        for (const auto& id : dxlHandler.dxl_ids) {
            printf("ID: %d, Current: %f mA, Velocity: %f, Position: %f \n", id, currents[id], velocities[id], positions[id]);
        }

        // 目標電流を設定
        double pos_f = positions[DXL_ID_FOLLOWER];
        double pos_l = positions[DXL_ID_LEADER];
        double vel_f = velocities[DXL_ID_FOLLOWER];
        double vel_l = velocities[DXL_ID_LEADER];
        double goal_current_l = KP * (pos_f - pos_l) + KD * (vel_f - vel_l);
        double goal_current_f = KP * (pos_l - pos_f) + KD * (vel_l - vel_f);

        map<int, double> goal_currents = {
            {DXL_ID_LEADER, goal_current_l},
            {DXL_ID_FOLLOWER, goal_current_f}
        };
        dxlHandler.setCurrents(goal_currents);

        usleep(2000); // Sleep for 0.002 seconds == 500Hz
    }


    dxlHandler.shutdown();

    return 0;
}