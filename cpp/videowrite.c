#include<opencv2/opencv.hpp>
#include<iostream>
#include<vector>
#include<stdio.h>
#include <sys/time.h>

int main(int argc, char *argv[])
{
    cv::Mat frame;
    cv::namedWindow("Cam", CV_WINDOW_NORMAL);

    cv::VideoCapture cap(1);
//    cv::VideoCapture cap("../street.mov");

//    cap.set(CV_CAP_PROP_FRAME_WIDTH,1280);
//    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 720); 

    int frame_width=    cap.get(CV_CAP_PROP_FRAME_WIDTH);
    int frame_height=   cap.get(CV_CAP_PROP_FRAME_HEIGHT);

    cv::VideoWriter video("video.avi",CV_FOURCC('M','J','P','G'),12, cv::Size(frame_width,frame_height),true);

    int i=0;
    for(;;i++)
    {
        cap >> frame;

        cv::imshow("Cam",frame);

        video.write(frame);

        char k = (char)cv::waitKey(1);
        if( k == 27 ) break;
    }
    return 0;
}