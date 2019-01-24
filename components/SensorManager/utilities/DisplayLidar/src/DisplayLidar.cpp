/*
 * DisplayLidar.cpp
 *
 *  Created on: 2019-01-22
 *      Author: liam
 */

/****************************************
 * INCLUDES
 ****************************************/

#include "Lidar.hpp"
#include "SensorManager.hpp"

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <lcm/lcm-cpp.hpp>
#include <string>
#include <chrono>
#include <iostream>

/****************************************
 * CONSTANTS
 ****************************************/

constexpr float MaxRange = 100.0;

/****************************************
 * Main
 ****************************************/

int main()
{
    SensorManager lidar;
    if(!lidar.Initialize(LIDAR_FLAG))
    {
        std::cout<<"[ERROR]: could not initialize lidar manager"<<std::endl;
        return -1;
    }

    lidar.Start();

    cv::Mat img(400, 400, CV_8UC1, cv::Scalar(0));
    cv::Point center(200,200);
    cv::namedWindow("Display window", cv::WINDOW_AUTOSIZE);

    while(lidar.IsRunning())
    {
        cv::Mat img(400, 400, CV_8UC1, cv::Scalar(0));
        lidar.Update();

        auto lidarData = lidar.GetLidar();

        const float resolution = (2.0*3.14159) / lidarData.numRanges;
        float angle = 0.0;

        const auto &scanline = lidarData.collapsedScanline;

        for(const auto &r : scanline.ranges)
        {
            const int y = 200 - (int)(2*r*cosf(angle));
            const int x = 200 + (int)(2*r*sinf(angle));
            cv::line(img, cv::Point(200,200), cv::Point(x,y), cv::Scalar(255), 2);
            angle += resolution;
        }
        cv::circle(img, center, 5, cv::Scalar(125), 3);

        cv::imshow( "Display window", img );
        cv::waitKey(1);
        img = cv::Scalar(0);
    }

    lidar.Stop();
}


