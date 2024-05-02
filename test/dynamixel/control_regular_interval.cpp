/*
    * control_regular_interval.cpp
    *
    *  一定間隔で制御を行うテストプログラム
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
#include <chrono>
#include <functional>


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
    // Initialize PortHandler instance
    // Set the port path
    // Get methods and members of PortHandlerLinux or PortHandlerWindows
    dynamixel::PortHandler* portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

    // Initialize PacketHandler instance
    // Set the protocol version
    // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
    dynamixel::PacketHandler* packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    // Open port
    if (portHandler->openPort()) {
        printf("Succeeded to open the port!\n");
    }
    else {
        printf("Failed to open the port!\n");
        printf("Press any key to terminate...\n");
        getch();
        return 0;
    }

    // Set port baudrate
    if (portHandler->setBaudRate(BAUDRATE)) {
        printf("Succeeded to change the baudrate!\n");
    }
    else {
        printf("Failed to change the baudrate!\n");
        printf("Press any key to terminate...\n");
        getch();
        return 0;
    }

    DXLHandler dxlHandler(portHandler, packetHandler);
    dxlHandler.addServo(DXL_ID_LEADER);
    dxlHandler.addServo(DXL_ID_FOLLOWER);
    dxlHandler.setup();

    int dxl_comm_result = COMM_TX_FAIL;             // Communication result

    map<int, double> currents, positions, velocities;

    function<void(int)> controller = [&](int interval) {
        while (1) {
            this_thread::sleep_for(chrono::milliseconds(interval)); // interval [ms]
            // 現在の状態を取得
            currents = dxlHandler.getCurrents();
            positions = dxlHandler.getPositions();
            velocities = dxlHandler.getVelocities();
            // for (const auto& id : dxlHandler.dxl_ids) {
            //     printf("ID: %d, Current: %f mA, Velocity: %f, Position: %f \n", id, currents[id], velocities[id], positions[id]);
            // }
        }
        };

    thread controller_thread(controller, 1000); // 1 Hz
    controller_thread.detach();

    while (1) {

        // // 現在の状態を取得
        // auto currents = dxlHandler.getCurrents();
        // auto positions = dxlHandler.getPositions();
        // auto velocities = dxlHandler.getVelocities();

        // 現在の状態を表示
        for (const auto& id : dxlHandler.dxl_ids) {
            printf("ID: %d, Current: %f mA, Velocity: %f, Position: %f \n", id, currents[id], velocities[id], positions[id]);
        }

        // // 目標電流を設定
        // double pos_f = positions[DXL_ID_FOLLOWER];
        // double pos_l = positions[DXL_ID_LEADER];
        // double vel_f = velocities[DXL_ID_FOLLOWER];
        // double vel_l = velocities[DXL_ID_LEADER];
        // double goal_current_l = KP * (pos_f - pos_l) + KD * (vel_f - vel_l);
        // double goal_current_f = KP * (pos_l - pos_f) + KD * (vel_l - vel_f);

        // map<int, double> goal_currents = {
        //     {DXL_ID_LEADER, goal_current_l},
        //     {DXL_ID_FOLLOWER, goal_current_f}
        // };
        // dxlHandler.setCurrents(goal_currents);

        // usleep(2000); // Sleep for 0.002 seconds == 500Hz
        this_thread::sleep_for(chrono::milliseconds(500)); // 0.5 ms
    }

    controller_thread.join();

    // Close port
    dxlHandler.shutdown();
    portHandler->closePort();

    return 0;
}