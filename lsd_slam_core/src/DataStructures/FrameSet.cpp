#include "DataStructures/FrameSet.h"
#include <iostream>

namespace lsd_slam {

FrameSet::FrameSet(std::array<Frame*, NUM_CAMERAS>& fs)
{
    for (int i=0; i<NUM_CAMERAS; i++) {
        frameSet[i] = fs[i];
        frameSet[i]->setFrameSet(this);
    }
    activeFrame = 0;
    //std::cout << "Made frame set \n";
}

void FrameSet::setActiveFrame(int af)
{
    activeFrame = af;
}

Frame *FrameSet::getActiveFrame()
{
    return frameSet[activeFrame];
}

}