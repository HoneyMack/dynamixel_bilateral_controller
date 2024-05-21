#include "cranex7_observer.hpp"

Cranex7Observer::Cranex7Observer(map<int, double> Js, map<int, double> Ds, map<int, double> Ms, double f0, double T) :j2_tau_r_filter(f0, T), j4_tau_r_filter(f0, T) {
    this->Js = Js;
    this->Ds = Ds;
    this->Ms = Ms;
    for (auto& kv : Js) {
        dobs[kv.first] = DOB(Js[kv.first], Ds[kv.first], f0, T);
        rfobs[kv.first] = RFOB(Js[kv.first], Ds[kv.first], f0, T);
    }
}

map<int, double> Cranex7Observer::step_torque_disturb(map<int, double> currents, map<int, double> positions, map<int, double> velocities) {
    map<int, double> tau_ds;
    for (auto& kv : currents) {
        tau_ds[kv.first] = dobs[kv.first].step(currents[kv.first], velocities[kv.first]);
    }
    return tau_ds;
}

map<int, double> Cranex7Observer::step_torque_react(map<int, double> currents, map<int, double> positions, map<int, double> velocities, map<int, double> tau_d) {
    map<int, double> tau_rs;
    for (auto& kv : currents) {
        // J2とJ4は例外処理
        if (kv.first == 2) {
            const double M1 = Ms[1], M2 = Ms[2];
            const double rad2 = positions[2]/180 * M_PI - M_PI, rad4 = positions[4]/180 * M_PI;

            const double tau_d_known = j2_tau_r_filter.filter(Ds[kv.first] * velocities[kv.first] - M1 * sin(rad2) + M2 * sin(rad2 + rad4));
            const double tau_r = tau_d[kv.first] - tau_d_known;
            tau_rs[kv.first] = tau_r;
        }
        else if (kv.first == 4) {
            const double M3 = Ms[3];
            const double rad4 = positions[4]/180 * M_PI ;

            const double tau_d_known = j4_tau_r_filter.filter(Ds[kv.first] * velocities[kv.first] + M3 * sin(rad4));
            const double tau_r = tau_d[kv.first] - tau_d_known;
            tau_rs[kv.first] = tau_r;
        }
        else {
            tau_rs[kv.first] = rfobs[kv.first].step(currents[kv.first], velocities[kv.first], tau_d[kv.first]);
        }
    }
    return tau_rs;
}