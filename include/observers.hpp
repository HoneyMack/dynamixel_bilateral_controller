#ifndef OBSERVERS_HPP
#define OBSERVERS_HPP

#include "filters.hpp"

// 外乱オブザーバ
class DOB {
protected:
    double J, D;
    LowPassFilter filter;
public:
    /// コンストラクタ
    DOB(): J(0), D(0), filter(0, 0) {} //デフォルトコンストラクタ
    DOB(double J, double D, double f0, double T) : J(J), D(D), filter(f0, T) {}

    double step(double current, double omega) {
        double x_next = filter.filter(current + J * filter.g * omega);
        double tau_d = x_next - J * filter.g * omega;
        return tau_d;
    }

    void reset() {
        filter.reset();
    }
};

// 反力オブザーバ
class RFOB {
protected:
    double J, D;
    LowPassFilter filter;
public:
    /// コンストラクタ
    RFOB(): J(0), D(0), filter(0, 0) {} //デフォルトコンストラクタ
    RFOB(double J, double D, double f0, double T) : J(J), D(D), filter(f0, T) {}

    double step(double current, double omega, double tau_d) {
        double tau_d_known = filter.filter(D * omega);
        double tau_r = tau_d - tau_d_known; //未知の外乱を推定
        return tau_r;
    }

    void reset() {
        filter.reset();
    }
};

#endif