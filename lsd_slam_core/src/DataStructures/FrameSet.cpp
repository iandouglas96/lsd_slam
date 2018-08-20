#include "DataStructures/FrameSet.h"
#include <iostream>

namespace lsd_slam {

FrameSet::FrameSet(std::array<std::shared_ptr<Frame>, NUM_CAMERAS>& fs)
{
    for (int i=0; i<NUM_CAMERAS; i++) {
        frameSet[i] = fs[i];
    }
    activeFrame = 0;
    //std::cout << "Made frame set \n";
}

std::array<std::weak_ptr<Frame>, NUM_CAMERAS> *FrameSet::getFrameSet()
{
    return &frameSet;
}

int FrameSet::getBestCamera()
{
    //Check all frames
	int bestCam = 0;
	int maxMappablePixels = 0;
	for (int i = 0; i<NUM_CAMERAS; i++) {
        std::shared_ptr<Frame> f = frameSet[i].lock();
		f->maxGradients(0);

        if (printInterestLevel) {
            std::cout << "Camera " << i << " has " << f->numMappablePixels << " mappable pixels\n";
        }

		if (f->numMappablePixels > maxMappablePixels) {
			maxMappablePixels = f->numMappablePixels;
            bestCam = i;
		}
	}

    return bestCam;
}

void FrameSet::setActiveFrame(int af)
{
    activeFrame = af;
}

std::shared_ptr<Frame> FrameSet::getFrame(int index)
{
    return frameSet[index].lock();
}

}