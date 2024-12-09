#ifndef _GET_ATTITUDE_
#define _GET_ATTITUDE_

#include <opencv2/opencv.hpp>
#include <algorithm>
#include <cmath>
#include <vector>

bool compareDouble(double &i,double &j) {return (i<j);}
bool compareLine(cv::Vec4i &line1,cv::Vec4i &line2)
{
        double slope1 = 0.0,slope2 = 0.0,angle1 = 0.0,angle2 = 0.0;
    
        slope1 = (line1[3] - line1[1])/(double)(line1[2] - line1[0]);
        slope2 = (line2[3] - line2[1])/(double)(line2[2] - line2[0]);
        angle1 = atan(slope1)*180/acos(-1.0);
        angle2 = atan(slope2)*180/acos(-1.0);

        return angle1 < angle2;
}

cv::Point2d getAttitude(std::vector<cv::Vec4i>& lines, cv::Point2d center)
{   
	cv::Point2d double_((double)361.0, (double)361.0);
    const double epsilon = 3.0;
    if(lines.size() == 0) return double_;
    std::vector<double> angles;
    std::vector<size_t> label(lines.size());
    cv::Point2d center2d((double)center.x, (double)center.y);

    for( size_t i = 0; i < lines.size(); i++ )
    {
        cv::Vec4i l = lines[i];
        double slope = 0.0,angle = 0.0;
        
        slope = (l[3] - l[1])/(double)(l[2] - l[0]);
        angle = atan(slope) * 180.0 / acos(-1.0);
        angles.push_back(angle);

        label[i] = 0;
    }
    std::sort(angles.begin(), angles.end(), compareDouble);
    std::sort(lines.begin(), lines.end(), compareLine);
    //#kinda clustering
    int index = 0;
    for(size_t i = 1; i != angles.size(); i++)
    {
        if(std::abs(angles[i] - angles[i-1]) > epsilon )
        {
            label[i] = ++index;
        }
        else
        {
            label[i] = index;
        }
    }
    std::vector<size_t> box(index+1);
    size_t labelOfMax = 0, tmpCounter = 0;
    for(size_t i = 0; i != box.size(); i++)
    { box[i] = 0; }
    for(size_t i = 0; i != label.size(); i++)
    { box[label[i]]++; }
    for(size_t i =0; i != box.size(); i++)
    {
        if(box[i] > tmpCounter)
        {
            tmpCounter = box[i];
            labelOfMax = i;
        }
    }
    std::vector<cv::Vec4i> lineOfInterest;
    double theta = 0.0;
    for(int i = 0; i != label.size(); i++)
    {
        if(label[i] == labelOfMax)
        {
            theta += angles[i];
            lineOfInterest.emplace_back(lines[i]);
        }
    }
    theta /= tmpCounter;
    cv::Vec4i L = lineOfInterest[0];
    cv::Point2d attitude = cv::Point2d(L[3]-L[1], L[0]-L[2]);
    cv::Point2d edge_to_center_vector;
    int inication = 0;
    for(int i = 0; i != lineOfInterest.size(); i++)
    {	
    	cv::Point2d edge(lineOfInterest[i][0], lineOfInterest[i][1]);
    	edge_to_center_vector = edge - center2d;
    	if( (edge_to_center_vector.x * attitude.x + edge_to_center_vector.y * attitude.y) > 0.0){
            inication++;
    	}
    	else{
            inication--;
    	}
    }
    if(inication >= 0){
    	attitude *= -1;
    }
    attitude /= hypot(attitude.x, attitude.y);
    return attitude;
}

#endif