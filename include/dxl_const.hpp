//dynamixel操作用の定数
#include <map>

#ifndef DXL_CONST_HPP
#define DXL_CONST_HPP

// Control table address
#define ADDR_OPERATING_MODE         11
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

#define BACKLASH_THREASHOLD        0.25 // [deg]


#define UNIT_VELOCITY               0.229 // [rev/min]
#define UNIT_POSITION               0.088 // [deg/pulse]


// DYNAMIXEL Protocol Version (1.0 / 2.0)
// https://emanual.robotis.com/docs/en/dxl/protocol2/
#define PROTOCOL_VERSION  2.0
#define BAUDRATE 1000000 // 1Mbps

enum class DynamixelType:int{
    XL330,
    XM430,
    XM540,
};

// 定数指定
static std::map<DynamixelType,const double> UNIT_CURRENT= // [mA]
    {
        {DynamixelType::XL330, 1.0},
        {DynamixelType::XM430, 2.69},
        {DynamixelType::XM540, 2.69},
    };

static std::map<DynamixelType,const double> MAX_CURRENT= // [mA]
    {
        {DynamixelType::XL330, 1700},
        {DynamixelType::XM430, 3200 - 50},
        {DynamixelType::XM540, 5500 - 50},
    };

static std::map<DynamixelType,const double> AMPERE_TO_TORQUE = // [rev/min]
    { 
        {DynamixelType::XM430, 1.8119* 1e-3},
        {DynamixelType::XM540, 2.2799* 1e-3},
    };

#endif