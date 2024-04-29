//dynamixel操作用の定数

#ifndef DXL_CONST_HPP
#define DXL_CONST_HPP

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

#define UNIT_CURRENT                1.0 // [mA]
#define UNIT_VELOCITY               0.229 // [rev/min]
#define UNIT_POSITION               0.088 // [deg/pulse]


#endif