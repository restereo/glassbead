#include <iostream>
#include <opencv2/opencv.hpp>
 
using namespace std;
using namespace cv;
 
int main (int argc, const char * argv[])
{
    VideoCapture cap(1);
//    cap.set(CV_CAP_PROP_FRAME_WIDTH, 1280);
//    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 720);    
    if (!cap.isOpened())
        return -1;
 
    Mat img;
    namedWindow("video capture");
    while (true)
    {
        cap >> img;
        if (!img.data)
            continue;
 
        imshow("video capture", img);
        if (waitKey(20) >= 0)
            break;
    }
    return 0;
}
