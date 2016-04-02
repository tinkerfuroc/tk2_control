#ifndef __TINKER_CONTORL_INTERPLOTE_H__
#define __TINKER_CONTORL_INTERPLOTE_H__

#include "tk_hardware_interface/common.h"
#include <vector>

namespace tinker {
namespace control {
class LinearInterpolation {
public:
    LinearInterpolation() {}
    LinearInterpolation(const std::vector<double> &xs,
                        const std::vector<double> &ys);
    double Get(double x);

private:
    std::vector<double> xs_;
    std::vector<double> ys_;
};
}
}
#endif
