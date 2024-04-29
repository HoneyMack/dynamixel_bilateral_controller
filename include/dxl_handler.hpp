// Dynamixelサーボの情報を取得するためのハンドラ

#ifndef DXL_HANDLER_HPP
#define DXL_HANDLER_HPP

#include <dynamixel_sdk/dynamixel_sdk.h>
#include <iostream>
#include <vector>
#include <map>

using namespace std;

class DXLHandler
{
    public:
        vector<int> dxl_ids;

        
        DXLHandler(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler);
        ~DXLHandler();

        void setup();
        void shutdown();
        void addServo(int id);
        void setTorqueEnable(int id, bool enable);
        void setCurrents(map<int,double> currents);
    
        map<int,double> getCurrents();
        map<int,double> getPositions();
        map<int,double> getVelocities();

    private:
        dynamixel::PortHandler *portHandler;
        dynamixel::PacketHandler *packetHandler;
        dynamixel::GroupSyncRead *currentSyncRead, *positionSyncRead, *velocitySyncRead;
        dynamixel::GroupSyncWrite *currentSyncWrite;


        bool checkError(int dxl_comm_result, uint8_t dxl_error, int id);
};

#endif // DXL_HANDLER_HPP