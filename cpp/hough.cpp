#include<opencv2/opencv.hpp>
#include<iostream>
#include<vector>
#include<stdio.h>
#include <sys/time.h>
#include <sys/types.h>

#include <unistd.h>

using namespace std;
using namespace cv;

struct timeval timer_st;

void timer_start() {
  gettimeofday(&timer_st, NULL);
}

void timer_log(const char *msg, bool show_fps=false) {
  struct timeval now;
  gettimeofday(&now, NULL);
  double tm = double(now.tv_sec - timer_st.tv_sec)+double(now.tv_usec - timer_st.tv_usec)/1000000.0;
  if (show_fps) {
    printf("%s: %0.3lf, fps: %0.1lf\n", msg, tm, 1.0/tm);
  } else {
    printf("%s: %0.3lf\n", msg, tm);
  }
}


//x0 <= x1

long BresenhamLineIntegrate(int x0, int y0, int x1, int y1, const Mat &dst) {

//  printf("ln: (%d,%d),(%d,%d)\n",x0,y0,x1,y1);

  long r=0;

  if (abs(x1-x0) > abs(y1-y0)) {
    //iterate by x
    if (x0>x1) {
      //swap ends
      int t = x0;x0 = x1;x1 = t;
      t=y0;y0=y1;y1=t;
    }
    int dx=x1-x0;
    int dy=y1-y0;

    int D = 2*dy - dx;
    r += (long)dst.at<uchar>(y0,x0);
    int y=y0;
    
    int dys = (dy==0)?0:(dy>0?1:-1);

    for(int x=x0+1;x<=x1;x++) {
      if (D > 0) {
        y = y+dys;
        r += (long)dst.at<uchar>(y,x);
        D = D + (2*dy-2*dx);
      } else {
        r += (long)dst.at<uchar>(y,x);
        D = D + (2*dy);        
      }
    }
    
  } else {
    //iterate by y;
    if (y0>y1) {
      //swap ends
      int t = x0;x0 = x1;x1 = t;
      t=y0;y0=y1;y1=t;
    }
    int dx=x1-x0;
    int dy=y1-y0;

    int D = 2*dx - dy;
    r += (long)dst.at<uchar>(y0,x0);
    int x=x0;
    
    int dxs = (dx==0)?0:(dx>0?1:-1);
//    printf("bl-y: dx=%d dy=%d dxs=%d D0=%d\n",dx, dy, dxs,D);
    for(int y=y0+1;y<=y1;y++) {
      if (D>0) {
        x = x+dxs;
        r += (long)dst.at<uchar>(y,x);
//        printf("  D=%d, pt(%d,%d)\n", D, x,y);
        D = D + (2*dx-2*dy);
      } else {
        r += (long)dst.at<uchar>(y,x);
//        printf("  D=%d, pt(%d,%d)\n",D, x,y);
        D = D + dxs*(2*dx);
      }
    }
  }
  return r;
}

void HoughDiscreteV( const Mat &img, int max_dx, int dbx, int ddx, Mat &dst ) {
  int i,j;
  
  CV_Assert( img.type() == CV_8UC1 );
  
  const uchar *image = img.ptr();
  int step = (int)img.step;
  int width = img.cols;
  int height = img.rows;
  
  printf("HD: width:%d height:%d\n",width,height);
  
  int num_dx = (2*max_dx + 1)/ddx+1;
  
  if (dst.empty()) {
    printf("dst_create: num_bx:%d, num_dx:%d\n",width/dbx+1, num_dx);
    dst.create( num_dx, width/dbx+1, CV_8UC1 );
  }    
  
  dst = Scalar::all(0.0);
  for(int basex=0; basex<width; basex+=dbx ) {
//    printf("basex: %d [%d,%d]\n",basex, max(0,basex-max_dx), min(basex+max_dx, width));
    for(int x=max(0, basex-max_dx); x<=min(basex+max_dx,width-1); x+=ddx) {

      float r = (float)BresenhamLineIntegrate(basex, 0, x, height-1, img);
      
      dst.at<uchar>( (max_dx + (x - basex))/ddx, basex/dbx) = r/(float)height/2.0; 
    }
  }
}

void HoughDiscreteV_( const Mat &img, int max_dx, int dbx, int ddx, Mat &dst) {
  int i,j;
  cv::AutoBuffer<short> _accum;
  
  CV_Assert( img.type() == CV_8UC1 );
  
  const uchar *image = img.ptr();
  int step = (int)img.step;
  int width = img.cols;
  int height = img.rows;
  
  printf("HD: width:%d height:%d\n",width,height);
  
  int num_bx = width/dbx + 1;
  int num_dx = (2*max_dx + 1)/ddx;
  
  if (dst.empty()) {
    printf("dst_create: num_bx:%d, num_dx:%d\n", num_bx, num_dx);
    dst.create( num_dx, num_bx, CV_32F );
  }    

  _accum.allocate((num_bx+2) * (num_dx+2));
  short *accum = _accum;
  memset( accum, 0, sizeof(accum[0]) * (num_bx+2) * (num_dx+2) );
  
  dst = Scalar::all(0);
  for(int y=0;y<height;y++) {
    for(int x=0;x<width;x++) {
      if (image[y*step+x] == 0) continue;

      //Bresenham? oh no?
/*
      int mdx = round((float)max_dx*y/height);
      
      int min_basex = max(0, x - mdx);
      min_basex = dbx*floor(min_basex/dbx);
      int max_basex = min(width-1, x + mdx);

//      printf("(x,y)=(%d,%d) => basex = [%d, %d]\n", x,y, min_basex, max_basex);       
*/
      float y_h = (float)y / (float)height;

      for(int _dx=0; _dx<num_dx; _dx+=1) {
        int basex = x + round((float)_dx * ddx * y_h);

        if (basex < 0) continue;
        if (basex >= width) continue;
 
        int bx = basex/dbx;

        accum[ (_dx+1)*(num_bx+2) + bx+1]++;       
//        dst.at<float>( (max_dx + dx) / ddx, basex / dbx)++;
      }           
    }
  }

/*
  for(int basex=0; basex<width; basex+=dbx ) {
//    printf("basex: %d [%d,%d]\n",basex, max(0,basex-max_dx), min(basex+max_dx, width));
    for(int x=max(0, basex-max_dx); x<=min(basex+max_dx,width-1); x+=ddx) {

      float r = (float)BresenhamLineIntegrate(basex, 0, x, height-1, img);
      
      dst.at<uchar>( (max_dx + (x - basex))/ddx, basex/dbx) = r/(float)height/2.0; 
    }
  }
*/
}




void HoughImg( const Mat& img, float rho, float theta, Mat& dst ) {
  int i, j;
  float irho = 1/rho;

  CV_Assert( img.type() == CV_8UC1 );

  const uchar* image = img.ptr();
  int step = (int)img.step;
  int width = img.cols;
  int height = img.rows;


  int numangle = cvRound( CV_PI / 2 / theta );
  int numrho = cvRound(((width+height) * 2 + 1) / rho);

  printf("numangle: %d, numrho: %d\n", numangle, numrho);

//  AutoBuffer<float> _accum((numangle+2) * (numrho+2));

  if (dst.empty()) {
    dst.create((numangle+2),(numrho+2), CV_32F);
  }

  dst = Scalar::all(0.0);

  float* accum = (float *)dst.ptr();
  int astep = dst.step;

  AutoBuffer<float> _tabSin(numangle);
  AutoBuffer<float> _tabCos(numangle);

  float *tabSin = _tabSin, *tabCos = _tabCos;

//  memset( accum, 0, sizeof(accum[0]) * (numangle + 2 ) * (numrho + 2) );
  float ang = 0; //min_theta;
  for(int n=0; n<numangle; ang+=theta, n++) {
    tabSin[n] = (float)(sin((double)ang)) * irho;
    tabCos[n] = (float)(cos((double)ang)) * irho;
  }
  printf("fill..\n");
  //fill accum
  for(i=0;i<height;i++)
    for(j=0;j<width;j++) {
      for (int n=0; n<numangle; n++) {
        int r = cvRound( j * tabCos[n] + i * tabSin[n] );
        r+= (numrho - 1) / 2;
        dst.at<float>(Point((r+1),(n+1))) += (float)(image[i*step+j])/(float)(255.0*1800*1.2);

        r = cvRound( - j * tabSin[n] + i * tabCos[n] );
        r = (numrho - 1) / 2 - r ;
        dst.at<float>(Point((r+1),(n+1))) += (float)(image[i*step+j])/(float)(255.0*1800*1.2);
      }
    }

  //find extremum
  float maxd = 0.00;
  int maxr=0, maxn = 0;

  float maxrhoex =0;
  int maxnex = 0;

  for(int n =0; n<numangle; n++) {
    int rhoex = 0;
    for(int r = 0; r<numrho; r++) {
      float d = dst.at<float>(Point(r+1,n+1));
      if ( (d>dst.at<float>(Point(r+2,n+1))) && (d>dst.at<float>(Point(r,n+1))) ) {
        rhoex++;
      }
    }
    if (rhoex > maxrhoex) {
      maxrhoex = rhoex;
      maxnex = n;
    }
  }
  printf("maxd=%0.3f, maxr=%d, maxn=%d | maxnex=%0.1f maxrhoex=%0.4f\n", maxd, maxr, maxn, (float)maxnex*90.0/(float)numangle, maxrhoex);
}

int main(int argc, char *argv[])
{
    cv::Mat frame, frame_;
    cv::Mat src_gray;
    cv::Mat dst, edges;

    cv::Mat output;

    cv::Mat frame_tm;
    cv::Mat eq_img;

//    cv::VideoCapture cap(1);
//    cap.set(CV_CAP_PROP_FRAME_WIDTH,1280);
    // cap.set(CV_CAP_PROP_FRAME_WIDTH,640);
//    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 720);
    // cap.set(CV_CAP_PROP_FRAME_HEIGHT, 480);


    cv::VideoCapture cap("video_goban_1280x960_1.avi");
//    cv::BackgroundSubtractorMOG2 bg(500, 0.89, false);
//    cv::BackgroundSubtractorMOG bg(500, 3, 0.7, 0.02);

    int frame_width=    cap.get(CV_CAP_PROP_FRAME_WIDTH);
    int frame_height=   cap.get(CV_CAP_PROP_FRAME_HEIGHT);

    printf("capture: %dx%d\n", frame_width, frame_height);

//    cv::VideoWriter video("mog.avi",CV_FOURCC('M','J','P','G'),12, cv::Size(frame_width,frame_height),true);

    std::vector<std::vector<cv::Point> > contours;

    cv::namedWindow("Frame");
//    cv::namedWindow("eq_img");
    cv::namedWindow("Canny", CV_WINDOW_NORMAL);
/*
    cv::namedWindow("Clouds", CV_WINDOW_NORMAL);
    cv::namedWindow("Cam", CV_WINDOW_NORMAL);
*/
    cv::moveWindow("Frame", 0,0);
    cv::resizeWindow("Frame", 640, 480);
    cv::moveWindow("Canny", 640,0);
    cv::resizeWindow("Canny", 640, 480);
/*
    cv::moveWindow("Fore", 1024, 260);
    cv::moveWindow("Clouds", 1024, 0);
    cv::moveWindow("Cam", 1024, 520);
*/
/*
    //prepare empty alpha frames
    if (fog_alpha.empty()) {
           fog_alpha.create(frame_height, frame_width, CV_8UC1);
    }
    fog_alpha = cv::Scalar::all(0);

    if (fore_alpha.empty()) {
           fore_alpha.create(frame_height, frame_width, CV_8UC1);
    }
    fore_alpha = cv::Scalar::all(0);
*/
    int i=0;
/*
    for(i=0;i<20;i++) {
        cap >> frame;
        cv::imshow("Frame",frame);
        if(cv::waitKey(30) >= 0) break;
    }
*/

/*
    bool update_bg_model = true;
    float y_crop_hist = 0.55*float(frame_height);

*/

    int lowTreshold = 25;
    float ratio = 3;
    int kernel_size = 3;

    float f_dr_avg = 0;


    for(;;i++)
    {
        timer_start();

        cap >> frame_;
        
        cv::resize(frame_, frame, Size(640,480));
        

        timer_log("time_capture");

        if (frame_tm.empty()) {
          frame_tm.create( frame.size(), frame.type() );
          frame.copyTo(frame_tm);
        } else {
          double t=0.20;
          addWeighted(frame, t, frame_tm, (1.0-t), 0, frame_tm);
        }

        if (dst.empty()) {
          dst.create( frame.size(), frame.type() );
        }
        frame.copyTo(dst);

        cvtColor( frame_tm, src_gray, CV_BGR2GRAY );


        blur( src_gray, edges, Size(3,3) );
//1.1: canny
//        Canny( edges, edges, lowTreshold, lowTreshold*ratio, kernel_size );

//1.2: sobel
        double scale = 1;
        double delta = 0;
        Mat edges_x, edges_y;


        Sobel( src_gray, edges_x, CV_16S, 1, 0, 3, scale, delta, BORDER_DEFAULT);
        Sobel( src_gray, edges_y, CV_16S, 0, 1, 3, scale, delta, BORDER_DEFAULT);

//        Scharr( src_gray, edges_x, CV_16S, 1, 0, scale, delta, BORDER_DEFAULT);
//        Scharr( src_gray, edges_y, CV_16S, 0, 1, scale, delta, BORDER_DEFAULT);

        Mat abs_grad_x, abs_grad_y;
        convertScaleAbs( edges_x, abs_grad_x );
        convertScaleAbs( edges_y, abs_grad_y );

        addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, edges);

        //for opencv/hough threshold is required
        cv::threshold(edges, edges, 35, 255, 0);

        int erosion_size = 2;
        Mat element =getStructuringElement(MORPH_ELLIPSE,
          Size(2*erosion_size+1,2*erosion_size+1),
          Point(erosion_size,erosion_size));
        cv:morphologyEx(edges, edges, MORPH_CLOSE, element);

            erosion_size = 1;
            element =getStructuringElement(MORPH_ELLIPSE,
          Size(2*erosion_size+1,2*erosion_size+1),
          Point(erosion_size,erosion_size));

        cv::morphologyEx(edges, edges, MORPH_OPEN, element);

        timer_log("time_prepare_image");
//2: hough

        int maxangle = 180; //360 => 0.5 degree angle step
        double rho=2.0;
        double theta = CV_PI/(float)maxangle;
        int threshold = 120;


/* Standart Hough

        vector<Vec2f> lines;
        HoughLines(edges, lines, rho, theta, threshold);
*/

        vector<Vec4i> lines;  /* minLineLength, maxLineMissingPart */
//        HoughLinesP(edges, lines, rho, theta, threshold, 270, 40);
//        printf("hough.lines: %d\n", lines.size());

        Mat dst_resize;
        cv::resize(frame_tm, dst_resize, Size(640,480));
        cv::imshow("Frame",dst_resize);

        Mat hough;

        HoughDiscreteV_(edges, 100, 2, 3, hough);

        double _max, _min;
        Point __max, __min;
        minMaxLoc(hough, &_min, &_max, &__min, &__max);
      
      printf("max: %2.2f\n", _max);
      
        convertScaleAbs( hough/(_max/255),hough );


      
        cv::resize(edges, dst_resize, Size(640,480));
        cv::imshow("Canny",dst_resize);

cv::imshow("Hough", hough);
        

        char k = (char)cv::waitKey(100);
        if( k == 27 ) break;
        if( k == ' ' )
        {
          fflush(stdout);
          while(true) {
            k = (char)cv::waitKey(100);
            if (k == ' ') break;
          }
        }
        timer_log("time_total", true);
    }
    return 0;

}