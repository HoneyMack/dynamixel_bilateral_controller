// dynamixelを管理するためのクラス．

#include "dxl_handler.hpp"
#include "dxl_const.hpp"
#include <iostream>
#include <fcntl.h>   // ファイル制御の定義
#include <errno.h>   // エラー番号の定義
#include <termios.h> // POSIX端末制御定義
#include <unistd.h>  // UNIX標準関数定義
#include <sys/ioctl.h> // 入出力制御の定義
#include <linux/serial.h> // Linuxシリアル通信定義

DXLHandler::DXLHandler(const char* device_name) {
    this->device_name = device_name;
    this->portHandler = dynamixel::PortHandler::getPortHandler(device_name);
    this->packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
}

DXLHandler::~DXLHandler() {
    //shutdown();
    delete currentSyncRead, positionSyncRead, velocitySyncRead;
    delete currentSyncWrite;
}


void DXLHandler::setup() {

    // Read, WriteHandlerの設定
    currentSyncRead = new dynamixel::GroupFastSyncRead(portHandler, packetHandler, ADDR_PRESENT_CURRENT, LEN_PRESENT_CURRENT);
    positionSyncRead = new dynamixel::GroupFastSyncRead(portHandler, packetHandler, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
    velocitySyncRead = new dynamixel::GroupFastSyncRead(portHandler, packetHandler, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY);

    currentSyncWrite = new dynamixel::GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_CURRENT, LEN_GOAL_CURRENT);

    // 一度シリアルポートを開いて，low_latencyモードにする
    int fd = open(device_name, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        std::cerr << "Failed to open the device." << std::endl;
        return;
    }
    else {
        struct serial_struct serial;
        if (ioctl(fd, TIOCGSERIAL, &serial) != -1) {
            serial.flags |= ASYNC_LOW_LATENCY;
            ioctl(fd, TIOCSSERIAL, &serial);
        }
        else {
            std::cerr << "Failed to get serial_struct." << std::endl;
        }
        close(fd);
    }


    // Open port
    if (portHandler->openPort()) {
        printf("Succeeded to open the port!\n");
    }
    else {
        printf("Failed to open the port!\n");
        return;
    }

    // Set port baudrate
    if (portHandler->setBaudRate(BAUDRATE)) {
        printf("Succeeded to change the baudrate!\n");
    }
    else {
        printf("Failed to change the baudrate!\n");
        printf("Press any key to terminate...\n");
        return;
    }


    // パラメータを追加
    for (const int dxl_id : this->dxl_ids) {
        int dxl_comm_result = currentSyncRead->addParam(dxl_id);
        if (dxl_comm_result != true) {
            fprintf(stderr, "[ID:%03d] groupBulkRead addparam failed\n", dxl_id);
        }
        dxl_comm_result = positionSyncRead->addParam(dxl_id);
        if (dxl_comm_result != true) {
            fprintf(stderr, "[ID:%03d] groupBulkRead addparam failed\n", dxl_id);
        }
        dxl_comm_result = velocitySyncRead->addParam(dxl_id);
        if (dxl_comm_result != true) {
            fprintf(stderr, "[ID:%03d] groupBulkRead addparam failed\n", dxl_id);
        }
    }
}

void DXLHandler::shutdown() {
    // 目標値電流を0にする
    map<int, double> currents;
    for (const int dxl_id : this->dxl_ids)
        currents[dxl_id] = 0;
    setCurrents(currents);

    // 必ずトルクをオフにする
    for (const int dxl_id : this->dxl_ids)
        setTorqueEnable(dxl_id, false);

    // Close port
    portHandler->closePort();
}

void DXLHandler::addServo(int id) {
    this->dxl_ids.push_back(id);
}

void DXLHandler::setTorqueEnable(int id, bool enable) {
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;


    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, id, ADDR_TORQUE_ENABLE, enable, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
        fprintf(stderr, "%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0) {
        fprintf(stderr, "%s\n", packetHandler->getRxPacketError(dxl_error));
    }
    else {
        string con_or_dis_message = enable ? "enabled" : "disabled";
        printf("Dynamixel ID:%03d has been successfully %s\n", id, con_or_dis_message.c_str());
    }
}

/// @brief サーボに加える電流を設定
/// @param currents 電流値[mA]
void DXLHandler::setCurrents(map<int, double> currents) {
    uint8_t param_goal_current[2];
    for (auto dxl_id : dxl_ids) {
        bool dxl_addparam_result = COMM_TX_FAIL;
        uint8_t dxl_error = 0;

        int16_t current = (int16_t)(currents[dxl_id] / UNIT_CURRENT);
        param_goal_current[0] = DXL_LOBYTE(current);
        param_goal_current[1] = DXL_HIBYTE(current);

        dxl_addparam_result = currentSyncWrite->addParam(dxl_id, param_goal_current);
        if (dxl_addparam_result != true) {
            fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed\n", dxl_id);
        }
    }
    currentSyncWrite->txPacket();
    currentSyncWrite->clearParam();
}

map<int, double> DXLHandler::getCurrents() {
    map<int, double> currents;
    // 現在の電流値を取得
    int dxl_comm_result = currentSyncRead->txRxPacket();

    // 受信できなければ早期リターン
    if (checkError(dxl_comm_result, 0, 0))
        return currents;

    for (const int dxl_id : dxl_ids) {
        int dxl_getdata_result = currentSyncRead->isAvailable(dxl_id, ADDR_PRESENT_CURRENT, LEN_PRESENT_CURRENT);
        if (dxl_getdata_result != true) {
            fprintf(stderr, "[ID:%03d] groupBulkRead getdata failed\n", dxl_id);
            continue;
        }
        int16_t current = currentSyncRead->getData(dxl_id, ADDR_PRESENT_CURRENT, LEN_PRESENT_CURRENT);
        currents[dxl_id] = current * UNIT_CURRENT;
    }

    return currents;
}


map<int, double> DXLHandler::getPositions() {
    map<int, double> positions;
    // 現在の位置を取得
    int dxl_comm_result = positionSyncRead->txRxPacket();

    // 受信できなければ早期リターン
    if (checkError(dxl_comm_result, 0, 0))
        return positions;

    for (const int dxl_id : dxl_ids) {
        int dxl_getdata_result = positionSyncRead->isAvailable(dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
        if (dxl_getdata_result != true) {
            fprintf(stderr, "[ID:%03d] groupBulkRead getdata failed\n", dxl_id);
            continue;
        }
        int32_t position = positionSyncRead->getData(dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
        positions[dxl_id] = position * UNIT_POSITION;
    }

    return positions;
}

map<int, double> DXLHandler::getVelocities() {
    map<int, double> velocities;
    // 現在の速度を取得
    int dxl_comm_result = velocitySyncRead->txRxPacket();

    // 受信できなければ早期リターン
    if (checkError(dxl_comm_result, 0, 0))
        return velocities;

    for (const int dxl_id : dxl_ids) {
        int dxl_getdata_result = velocitySyncRead->isAvailable(dxl_id, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY);
        if (dxl_getdata_result != true) {
            fprintf(stderr, "[ID:%03d] groupBulkRead getdata failed\n", dxl_id);
            continue;
        }
        int32_t velocity = velocitySyncRead->getData(dxl_id, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY);
        velocities[dxl_id] = velocity * UNIT_VELOCITY;
    }

    return velocities;
}


bool DXLHandler::checkError(int dxl_comm_result, uint8_t dxl_error, int id) {
    if (dxl_comm_result != COMM_SUCCESS) {
        fprintf(stderr, "%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        return true;
    }
    else if (dxl_error != 0) {
        fprintf(stderr, "%s\n", packetHandler->getRxPacketError(dxl_error));
        return true;
    }
    else {
        return false;
    }
}