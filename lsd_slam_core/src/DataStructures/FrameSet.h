#pragma once
#include "DataStructures/Frame.h"

namespace lsd_slam {

class Frame;
class FrameSet {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    FrameSet(std::array<std::shared_ptr<Frame>, NUM_CAMERAS>& fs);

    void setActiveFrame(int af);
    std::shared_ptr<Frame> getFrame(int index);
    std::array<std::weak_ptr<Frame>, NUM_CAMERAS> *getFrameSet();
    int getBestCamera();
private:
    std::array<std::weak_ptr<Frame>, NUM_CAMERAS> frameSet;
    int activeFrame;
};

}