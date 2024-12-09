#ifndef _GETSAFERECT_
#define _GETSAFERECT_

#include <opencv2/opencv.hpp>

cv::Rect getSafeRect(int &width, int &height, cv::Rect const rect)
{
	int x = rect.x - 30;
	int y = rect.y - 30;
	int w = rect.width + 60;
	int h = rect.height + 60;

	if(x < 0)
		x = 1;
	if(y < 0)
		y = 1;
	if((x+w) > width)
		w = width - x -1;
	if((y+h) > height)
		h = height - y -1;

	cv::Rect outRect(x,y,w,h);
	return outRect;
}

#endif