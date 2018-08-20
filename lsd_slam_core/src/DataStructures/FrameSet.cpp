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

std::array<Frame*, NUM_CAMERAS> *FrameSet::getFrameSet()
{
    return &frameSet;
}

int FrameSet::getBestCamera()
{
    //Check all frames
	int bestCam = 0;
	int maxMappablePixels = 0;
	for (int i = 0; i<NUM_CAMERAS; i++) {
		Frame *f = frameSet[i];
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

Frame *FrameSet::getActiveFrame()
{
    return frameSet[activeFrame];
}

}