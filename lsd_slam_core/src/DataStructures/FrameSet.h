#pragma once
#include "DataStructures/Frame.h"

namespace lsd_slam {

class Frame;
class FrameSet {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    FrameSet(std::array<Frame*, NUM_CAMERAS>& fs);

    void setActiveFrame(int af);
    Frame *getActiveFrame();
    std::array<Frame*, NUM_CAMERAS> *getFrameSet();
    int getBestCamera();
private:
    std::array<Frame*, NUM_CAMERAS> frameSet;
    int activeFrame;
};

}