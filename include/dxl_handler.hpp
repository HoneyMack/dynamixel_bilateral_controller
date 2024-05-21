// Dynamixelサーボの情報を取得するためのハンドラ

#ifndef DXL_HANDLER_HPP
#define DXL_HANDLER_HPP

#include <dynamixel_sdk/dynamixel_sdk.h>
#include "dxl_const.hpp"
#include <iostream>
#include <vector>
#include <map>


using namespace std;
using dxlType = DynamixelType; // 名前が長いので省略して使用


class DXLHandler
{
    public:
        vector<int> dxl_ids;

        DXLHandler(const char* device_name, const int baudrate);
        ~DXLHandler();

        void setup(bool torque_enable = true, int mode = -1);
        void shutdown();
        void setOperationMode(int dxl_id, int mode);
        void addServo(int id, dxlType type);
        void setTorqueEnable(int id, bool enable);
        void setCurrents(map<int,double> currents);
    
        map<int,double> getCurrents();
        map<int,double> getPositions();
        map<int,double> getVelocities();

    private:
        const char* device_name;
        const int baudrate;
        map<int, dxlType> dxl_types;
        dynamixel::PortHandler *portHandler;
        dynamixel::PacketHandler *packetHandler;
        dynamixel::GroupFastSyncRead *currentSyncRead, *positionSyncRead, *velocitySyncRead;
        dynamixel::GroupSyncWrite *currentSyncWrite;


        bool checkError(int dxl_comm_result, uint8_t dxl_error, int id);
};

#endif // DXL_HANDLER_HPP