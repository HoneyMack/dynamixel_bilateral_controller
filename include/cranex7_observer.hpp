#ifndef CRANEX7_OBSERVER_HPP
#define CRANEX7_OBSERVER_HPP

#include <map>
#include <string>
#include <tuple>
#include "observers.hpp"
#include "dxl_const.hpp"

using namespace std;

// cranex7observerに与えられるmapのidx(first index)は，「Imitation Learning for Nonprehensile Manipulation through Self-Supervised Learning Considering Motion Speed」のjoint_idxに一致していると仮定



class Cranex7Observer {
protected:
    map<int, double> Js;
    map<int, double> Ds;
    map<int, double> Ms;
    map<int, DOB> dobs;
    map<int, RFOB> rfobs;
    map<int, double> before_positions;
    map<int, int> move_directions; // 1: CW, -1: CCW, 0: stop
    LowPassFilter j2_tau_r_filter, j4_tau_r_filter;

public:
    Cranex7Observer(map<int, double> Js, map<int, double> Ds, map<int, double> Ms, double f0, double T);

    map<int, double> step_torque_disturb(map<int, double> currents, map<int, double> positions, map<int, double> velocities);
    map<int, double> step_torque_react(map<int, double> currents, map<int, double> positions, map<int, double> velocities, map<int, double> tau_d);
};


#endif