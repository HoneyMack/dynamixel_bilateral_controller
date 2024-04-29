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


#include "dynamixel_sdk.h"  // Uses DYNAMIXEL SDK library
#include <unistd.h>

using namespace std;

/********* DYNAMIXEL Model definition *********
***** (Use only one definition at a time) *****/


// Control table address
#define ADDR_TORQUE_ENABLE          64
#define ADDR_GOAL_CURRENT           102
#define LEN_GOAL_CURRENT            2
#define ADDR_GOAL_POSITION          116
#define LEN_GOAL_POSITION           4
#define ADDR_PRESENT_CURRENT        126 // 1 [mA]
#define LEN_PRESENT_CURRENT         2
#define ADDR_PRESENT_VELOCITY       128 // 0.229 [rev/min]
#define LEN_PRESENT_VELOCITY        4
#define ADDR_PRESENT_POSITION       132 // 0.088 [deg/pulse]
#define LEN_PRESENT_POSITION        4
#define MINIMUM_POSITION_LIMIT      0  // Refer to the Minimum Position Limit of product eManual
#define MAXIMUM_POSITION_LIMIT      4095  // Refer to the Maximum Position Limit of product eManual

// DYNAMIXEL Protocol Version (1.0 / 2.0)
// https://emanual.robotis.com/docs/en/dxl/protocol2/
#define PROTOCOL_VERSION  2.0


// SERVO_ID
#define DXL_ID_LEADER 1
#define DXL_ID_FOLLOWER 2
#define BAUDRATE                    57600
#define DEVICENAME  "/dev/ttyUSB0"


// Helper constants
#define TORQUE_ENABLE                   1
#define TORQUE_DISABLE                  0
#define DXL_MOVING_STATUS_THRESHOLD     20  // DYNAMIXEL moving status threshold
#define ESC_ASCII_VALUE                 0x1b


const double KP = 0.1;
const double KD = 1e-8;

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

int kbhit(void) {
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if (ch != EOF) {
        ungetc(ch, stdin);
        return 1;
    }

    return 0;
}



int main() {
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    bool dxl_addparam_result = false;               // addParam result
    bool dxl_getdata_result = false;                // GetParam result

    auto dxl_ids = { DXL_ID_LEADER, DXL_ID_FOLLOWER }; // DYNAMIXEL ID List
    uint8_t dxl_error = 0;                          // Dynamixel error
    uint8_t param_goal_current[LEN_PRESENT_CURRENT];

    map<int8_t, int16_t> dxl_goal_currents;
    map<int8_t, int32_t> dxl_present_currents;
    map<int8_t, int32_t> dxl_present_velocities;
    map<int8_t, int32_t> dxl_present_positions;

    vector<pair<int, int>> read_params = { {ADDR_PRESENT_CURRENT, LEN_PRESENT_CURRENT}, {ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY}, {ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION} };
    vector<map<int8_t, int32_t>*> read_values = { &dxl_present_currents, &dxl_present_velocities, &dxl_present_positions };

    // Initialize PortHandler instance
    // Set the port path
    // Get methods and members of PortHandlerLinux or PortHandlerWindows
    dynamixel::PortHandler* portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

    // Initialize PacketHandler instance
    // Set the protocol version
    // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
    dynamixel::PacketHandler* packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    // Initialize GroupBulkWrite instance
    dynamixel::GroupBulkWrite groupBulkWrite(portHandler, packetHandler);

    // Create GroupBulkRead instances for each read parameter
    vector<dynamixel::GroupBulkRead> groupBulkReads;

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

    // Enable DYNAMIXELs Torque
    for (auto&& dxl_id : dxl_ids) {
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) {
            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0) {
            printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        }
        else {
            printf("Succeeded enabling DYNAMIXEL Torque.\n");
        }
    }

    // Add parameter storage for Dynamixels present current,velocity and position
    // TODO: なぜかemplace_backを下のfor文の中で使うとエラーが出るので別々にしている．要原因調査
    for(int rparam_idx = 0; rparam_idx < read_params.size(); rparam_idx++){
        groupBulkReads.emplace_back(portHandler, packetHandler);
    }

    for (int rparam_idx = 0; rparam_idx < read_params.size();rparam_idx++){
        const auto [addr, len] = read_params[rparam_idx];

        for (const auto dxl_id : dxl_ids) {
            bool dxl_addparam_result = groupBulkReads[rparam_idx].addParam(dxl_id, addr, len);
            if (!dxl_addparam_result) {
                fprintf(stderr, "[ID:%03d] groupBulkRead addparam failed\n", dxl_id);
                return 0;
            }
            else {
                printf("[ID:%03d] groupBulkRead addparam success\n", dxl_id);
            }
        }
    }

    while (1) {
        // 現在の状態を取得
        //// 受信できたか確認
        for (int rparam_idx = 0; rparam_idx < read_params.size(); rparam_idx++) {
            printf("Succeeded to get present position\n");
            dxl_comm_result = groupBulkReads[rparam_idx].txRxPacket();
            if (dxl_comm_result != COMM_SUCCESS) {
                printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
                break;
            }
        }
        //// 受信データを取得 
        for (int rparam_idx = 0; rparam_idx < read_params.size(); rparam_idx++) {
            // TODO: エラー処理を追加
            for (auto&& dxl_id : dxl_ids) {
                auto& [addr, len] = read_params[rparam_idx];
                auto val = read_values[rparam_idx];

                dxl_getdata_result = groupBulkReads[rparam_idx].isAvailable(dxl_id, addr, len);
                if (dxl_getdata_result != true) {
                    fprintf(stderr, "[ID:%03d] groupBulkRead getdata failed\n", dxl_id);
                    return 0;
                }
                (*val)[dxl_id] = groupBulkReads[rparam_idx].getData(dxl_id, addr, len);
            }
        }

        // 現在の情報を表示
        for (auto&& dxl_id : dxl_ids) {
            printf("[ID:%03d] Present Current: %d mA, Present Velocity: %d rpm, Present Position: %d\n", dxl_id, dxl_present_currents[dxl_id], dxl_present_velocities[dxl_id], dxl_present_positions[dxl_id]);
        }

        // フィードバック制御
        // 目標電流値を設定


        dxl_goal_currents[DXL_ID_LEADER] = KP * (dxl_present_positions[DXL_ID_FOLLOWER] - dxl_present_positions[DXL_ID_LEADER]) + KD * (dxl_present_velocities[DXL_ID_FOLLOWER] - dxl_present_velocities[DXL_ID_LEADER]);
        dxl_goal_currents[DXL_ID_FOLLOWER] = KP * (dxl_present_positions[DXL_ID_LEADER] - dxl_present_positions[DXL_ID_FOLLOWER]) + KD * (dxl_present_velocities[DXL_ID_LEADER] - dxl_present_velocities[DXL_ID_FOLLOWER]);
        
        //パラメータを追加
        for(auto dxl_id:dxl_ids){
            param_goal_current[0] = DXL_LOBYTE((dxl_goal_currents[dxl_id]));
            param_goal_current[1] = DXL_HIBYTE((dxl_goal_currents[dxl_id]));

            dxl_addparam_result = groupBulkWrite.addParam(dxl_id, ADDR_GOAL_CURRENT, LEN_GOAL_CURRENT, param_goal_current);
            if (dxl_addparam_result != true) {
                fprintf(stderr, "[ID:%03d] groupBulkWrite addparam failed\n", dxl_id);
                return 0;
            }
        }
        // write goal current
        dxl_comm_result = groupBulkWrite.txPacket();
        if (dxl_comm_result != COMM_SUCCESS) {
            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        }

        // Clear bulkwrite parameter storage
        groupBulkWrite.clearParam();

        usleep(10000); // Sleep for 0.01 seconds
        //usleep(1000000); // Sleep for 0.1 seconds


        // printf("Press any key to continue! (or press ESC to quit!)\n");
        // if (getch() == ESC_ASCII_VALUE)
        //     break;
    }

    // Disable DYNAMIXEL Torque
    for (auto&& dxl_id : dxl_ids) {
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) {
            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0) {
            printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        }
        else {
            printf("Succeeded disabling DYNAMIXEL Torque.\n");
        }
    }

    // Close port
    portHandler->closePort();
    return 0;
}