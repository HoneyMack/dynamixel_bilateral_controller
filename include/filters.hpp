#ifndef FILTERS_HPP
#define FILTERS_HPP

#include <cmath>

class LowPassFilter {
private:
    double y;  // filtered output
    double a;  // filter coefficient


public:
    double g; //カットオフ角周波数
    // Constructor
    LowPassFilter(double f0, double T) : y(0) {
        g = 2 * M_PI * f0;
        a = 1 / (1 + g * T);
    }

    // Filter a value
    double filter(double x) {
        y = a * y + (1 - a) * x;
        return y;
    }

    // Reset the filter
    void reset() {
        y = 0;
    }
};

class FilterdDifferentiator {
private:
    double y;  // filtered output
    double a;  // filter coefficient
    double x_prev;
public:
    double g; //カットオフ角周波数
    // Constructor
    FilterdDifferentiator(double f0, double T) : y(0), x_prev(0) {
        g = 2 * M_PI * f0;
        a = 1 / (1 + g * T);
    }

    // Filter a value
    double filter(double x) {
        y = a * y + a * g * (x - x_prev);
        x_prev = x;
        return y;
    }

    // Reset the filter
    void reset() {
        y = 0;
    }
};



#endif