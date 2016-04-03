#include "tk_hardware_interface/interpolate.h"
#include <ros/ros.h>

namespace tinker {
namespace control {

using std::vector;

LinearInterpolation::LinearInterpolation(const vector<double> &xs,
                                         const vector<double> &ys)
    : xs_(xs), ys_(ys) {
    ROS_ASSERT(xs.size() == ys.size());
    ROS_ASSERT(xs.size() > 0);
}

double LinearInterpolation::Get(double x) {
    if (x <= xs_[0]) return ys_[0];
    if (x >= xs_.back()) return ys_.back();
    int lo = 0;
    int hi = xs_.size();
    while(lo < hi - 1) {
        int mid = (lo + hi) / 2;
        if (xs_[mid] < x) lo = mid; 
        else hi = mid;
    }
    double x_lo = xs_[lo];
    double x_hi = xs_[hi];
    double y_lo = ys_[lo];
    double y_hi = ys_[hi];
    double weight_lo = (x_hi - x) / (x_hi - x_lo);
    double weight_hi = (x - x_lo) / (x_hi - x_lo);
    return weight_lo * y_lo + weight_hi * y_hi;
}

}
}
