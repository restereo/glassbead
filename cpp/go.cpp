#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <stdio.h>
#include <sys/time.h>

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

Vec2f toPolar(Vec4i l) {
    float ang = -atan2((float)(l[2]-l[0]), (float)(l[3]-l[1]));
    float r = ((float)(l[1])*sin((double)ang) + (float)l[0]*cos((double)ang));
    return Vec2f(r, ang);
}

bool waytosort(Vec2f a, Vec2f b) {
  return a[0] > b[0];
}

void polarLine(Mat dst, Vec2f lp) {

  //draw
  double a = cos(lp[1]);
  double b = sin(lp[1]);

  double x0 = a*lp[0], y0 = b*lp[0];
  Point pt1(cvRound(x0 + 1000*(-b)), cvRound(y0 + 1000*(a)));
  Point pt2(cvRound(x0 - 1000*(-b)), cvRound(y0 - 1000*(a)));

  line(dst, pt1, pt2, Scalar(0,255,0), 1, 8 );

}

int findBestAngle(vector<Vec4i> lines, int numangle) {

  AutoBuffer<int> _counts(numangle);
  int *counts = _counts;
  memset(counts, 0, numangle * sizeof(int));

  for( size_t j=0; j<lines.size(); j++) {
    Vec4i l = lines[j];
    Vec2f lp = toPolar(l);

    float ang = lp[1]/CV_PI*180;

    if (ang < 0) ang+=180;
    if (ang >= 180) ang-=180;

    // 0 to 90 deg
    if (ang >=90 ) ang -=90;

    int _ang = cvRound(ang);
    if ((_ang>=0) && (_ang<90)) {
      counts[_ang] ++;
    }
  }

  int maxA = 0;
  int maxAc = 0;
  for ( int a=0; a<numangle/2; a++) {
      if (counts[a] > maxAc) {
        maxAc = counts[a];
        maxA = a;
      }
  }
  printf("maxA: %d\n", maxA);
  return maxA;
}

vector<Vec2f> findHLines(vector<Vec4i> lines, int ang_threshold, int maxA) {

  vector<Vec2f> hlines;

  for( size_t j=0; j<lines.size(); j++) {

    Vec4i l = lines[j];
    Vec2f lp = toPolar(l);

    float ang = lp[1];
    float r = lp[0];

    ang = ang/CV_PI*180;

    if (ang < 0) ang+=180;

    float a = abs(maxA - ang);
    float aa= abs(180 + maxA - ang);
    if (a < ang_threshold || aa <ang_threshold) {
      hlines.push_back( lp );
    }

  }

  sort(hlines.begin(), hlines.end(), waytosort);

  return hlines;
}

vector<Vec2f> filterHLines(vector<Vec2f> hlines, int merge_dist) {

  vector<Vec2f> hlines2;

  Vec2f last_lp = Vec2f(0,0);
  int last_count = 0;

  for (size_t j=0;j<hlines.size();j++) {

    Vec2f lp = hlines[j];

    if (last_count == 0) {
      last_lp[0] = lp[0];
      last_lp[1] = lp[1];
      last_count++;
      continue;
    }

    if (abs(last_lp[0] - lp[0]) <= merge_dist) {
      last_lp = (last_lp*(float)last_count + lp) / (float)(last_count+1);
      last_count++;
    } else {
      hlines2.push_back(last_lp);
      last_count = 0;
    }
  }

  if (last_count) {
    hlines2.push_back(last_lp);
  }

  return hlines2;
}

void dumpLines(vector<Vec2f> lines) {

  Vec2f prev_line;

  for(size_t j=0;j<lines.size();j++) {

    Vec2f lp = lines[j];

    float ang = lp[1];
    float r   = lp[0];

    ang = ang/CV_PI*180;

    if (ang < 0) ang+=180;
    if (ang >= 180) ang-=180;

    if (j == 0) {
      printf("ang: %1.f, r=%1.f\n", ang, r);
    } else {
      printf("ang: %1.f, r=%1.f, dr=%1.f\n", ang, r, abs(prev_line[0]-r));
    }

    prev_line = lp;
  }
}

void dumpLines(vector<Vec4i*> lines) {

  for( size_t j=0; j<lines.size(); j++) {
    Vec4i l = *(lines[j]);
    Vec2f lp = toPolar(l);

    float ang = lp[1];
    float r = lp[0];

    ang = ang/CV_PI*180;

    if (ang < 0) ang+=180;
    if (ang >= 180) ang-=180;

    printf("4i: r=%0.f ang=%0.f\n", r, ang);

  }
}

Point intersect(Vec2f a, Vec2f b) {

   float x1 = a[0]*cos(a[1]);
   float y1 = a[0]*sin(a[1]);

   float x2 = b[0]*cos(b[1]);
   float y2 = b[0]*sin(b[1]);

   float t2 = ((y2-y1)*sin(a[1]) + (x2-x1)*cos(a[1])) / (cos(b[1])*sin(a[1]) - sin(b[1])*cos(a[1]));

   float x = x2 + t2*sin(b[1]);
   float y = y2 - t2*cos(b[1]);

   printf("(x,y) = (%1.f, %1.f)\n", x, y);
   return Point(cvRound(x), cvRound(y));
}

float median(vector<float> &v) {

    size_t n = v.size() / 2;
    nth_element(v.begin(), v.begin()+n, v.end());
    return v[n];
}

// Устаканенная решетка
vector<Vec2f> _hbars;
vector<Vec2f> _vbars;

// функция для построения решетки по отфильтрованным линиям
vector<Vec2f> setBars(vector<Vec2f> hlines2) { // , vector<Vec2f> vlines2

  // предполагаем, что верхняя и нижняя линии распознаны правильно и достроим решетку по ним, посчитав средний dr

  vector<float> _h_drs;
  float _prev_r;
  _prev_r = 0.0;

  for (int i = 0; i < hlines2.size(); ++i) {

    float r = hlines2[i][0];
    _h_drs.push_back(abs(r - _prev_r));
  }

  float m_dr = median(_h_drs);
  printf("median dr: %1.f\n", m_dr);
  // sort(_h_drs.begin(), _h_drs.end());

  std::vector<int> _lines_to_add;
  std::vector<int> _lines_to_remove;

  float magic = 1.5;

  if (hlines2.size() < 21) { // надо добавить линий

    for (int i = 0; i < _h_drs.size(); ++i) {

      if (_h_drs[i] > m_dr * magic) {

        _lines_to_add.push_back(i);
      }
    }
  } else { // надо убрать лишние линии
    for (int i = 0; i < _h_drs.size(); ++i) {
      if (_h_drs[i] < m_dr/magic) {

        _lines_to_remove.push_back(i);
      }
    }
  }

  for (int i = 0; i < _lines_to_add.size(); ++i) {
    hlines2.push_back(Vec2f(hlines2[i][0] + m_dr, hlines2[i][1]));
  }

  for (int i = 0; i < _lines_to_remove.size(); ++i) {
    hlines2.erase (hlines2.begin()+i);
  }

  sort(hlines2.begin(), hlines2.end(), waytosort);

  return hlines2;

}


int main(int argc, char *argv[])
{
    cv::Mat frame;
    cv::Mat src_gray;
    cv::Mat dst, edges;

    cv::Mat output;

    cv::Mat frame_tm;

//    cv::VideoCapture cap(1);
//    cap.set(CV_CAP_PROP_FRAME_WIDTH,1280);
//    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 720);


    cv::VideoCapture cap("./video.avi");
//    cv::BackgroundSubtractorMOG2 bg(500, 0.89, false);
    cv::BackgroundSubtractorMOG bg(500, 3, 0.7, 0.02);

    int frame_width=    cap.get(CV_CAP_PROP_FRAME_WIDTH);
    int frame_height=   cap.get(CV_CAP_PROP_FRAME_HEIGHT);

    printf("capture: %dx%d\n", frame_width, frame_height);

    cv::VideoWriter video("mog.avi",CV_FOURCC('M','J','P','G'),12, cv::Size(frame_width,frame_height),true);

    std::vector<std::vector<cv::Point> > contours;

    cv::namedWindow("Frame");
    cv::namedWindow("Canny", CV_WINDOW_NORMAL);
/*
    cv::namedWindow("Clouds", CV_WINDOW_NORMAL);
    cv::namedWindow("Cam", CV_WINDOW_NORMAL);
*/
    cv::moveWindow("Frame", 0,0);
    cv::resizeWindow("Frame", frame_width, frame_height);
    cv::moveWindow("Canny", 640,0);
    cv::resizeWindow("Canny", frame_width, frame_height);
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

    for(;;i++)
    {
        timer_start();

        cap >> frame;

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
        frame_tm.copyTo(dst);

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

       // Scharr( src_gray, edges_x, CV_16S, 1, 0, scale, delta, BORDER_DEFAULT);
       // Scharr( src_gray, edges_y, CV_16S, 0, 1, scale, delta, BORDER_DEFAULT);

        Mat abs_grad_x, abs_grad_y;
        convertScaleAbs( edges_x, abs_grad_x );
        convertScaleAbs( edges_y, abs_grad_y );

        addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, edges);

        //for opencv/hough threshold is required
        cv::threshold(edges, edges, 30, 255, 0);



//2: hough

        double rho=2.0;
        double theta = CV_PI/180.0;
        int threshold = 100;
/*
        printf("call houghimg\n");
        HoughImg(edges, rho, theta, dst);

*//*
        vector<Vec2f> lines;
        HoughLines(edges, lines, rho, theta, threshold);

        printf("lines.size: %d\n", lines.size());
        for( size_t j=0; j<lines.size(); j++ ){
            float rho   = lines[j][0];
            float theta = lines[j][1];

            printf("line[%05d] rho=%0.1f, theta=%0.1f\n", j, rho, theta/CV_PI*180);

            Point pt1, pt2;
            double a = cos(theta), b=sin(theta);
            double x0 = a*rho, y0 = b*rho;
            pt1.x = cvRound(x0+1000*(-b));
            pt1.y = cvRound(y0+1000*(a));
            pt2.x = cvRound(x0-1000*(-b));
            pt2.y = cvRound(y0-1000*(a));

            line(dst, pt1, pt2, Scalar(0,0,255), 1, CV_AA);

        }
*/

        vector<Vec4i> lines;
        HoughLinesP(edges, lines, rho, theta, 150, 150, 30);
        printf("hough.lines: %d\n", (int) lines.size());

/*
        for(size_t j=0; j<lines.size(); j++) {
          Vec4i l = lines[j];
          line(dst, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255,0,0), 1, 8);
        }
*/
        int maxA = findBestAngle(lines, 180); //180/2 degree values

        vector<Vec2f> hlines = findHLines(lines, 8, maxA);
        vector<Vec2f> vlines = findHLines(lines, 8, maxA+90);


        printf("lines: h.size=%d v.size=%d\n", (int)hlines.size(), (int)vlines.size());

        vector<Vec2f> hlines2 = filterHLines(hlines, 12);
        vector<Vec2f> vlines2 = filterHLines(vlines, 7);

        // hlines2 = setBars(hlines2);

        for(size_t j=0; j<hlines2.size(); j++) polarLine(dst, hlines2[j]);
        for(size_t j=0; j<vlines2.size(); j++) polarLine(dst, vlines2[j]);


        printf("lines2: h.size=%d v.size=%d\n", hlines2.size(), vlines2.size());

        dumpLines(hlines2);

        if ((hlines2.size() == 21) && (vlines2.size() == 21)) {

          Point pt1 = intersect(hlines2[1], vlines2[1]);
          Point pt2 = intersect(hlines2[1], vlines2[vlines2.size()-2]);
          Point pt3 = intersect(hlines2[hlines2.size()-2], vlines2[1]);
          Point pt4 = intersect(hlines2[hlines2.size()-2], vlines2[vlines2.size()-2]);

          line(dst, pt1, pt2, Scalar(0,0,255), 2, 8 );
          line(dst, pt2, pt4, Scalar(0,0,255), 2, 8 );
          line(dst, pt3, pt4, Scalar(0,0,255), 2, 8 );
          line(dst, pt3, pt1, Scalar(0,0,255), 2, 8 );


        }

/*

        int maxdr = 30;
        AutoBuffer<int> _dritems(maxdr);
        memset(_dritems, 0, sizeof(int)*(maxdr));

        float prev_r;
        for(size_t j=0;j<vlines2.size();j++) {
          if (j!=0) {
            int dr = cvRound(abs(prev_r - vlines2[j][0]));
            if (dr<maxdr) {
              _dritems[dr]++;
            }
          }
          prev_r = vlines2[j][0];
        }

        for(size_t j=0;j<hlines2.size();j++) {
          if (j!=0) {
            int dr = cvRound(abs(prev_r - hlines2[j][0]));
            if (dr<maxdr) {
              _dritems[dr]++;
            }
          }
          prev_r = hlines2[j][0];
        }

        for(size_t j=0; j< maxdr; j++) {
          printf("dr=%d, count=%d\n", j, _dritems[j]);
        }
*/

//            line(dst, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255,0,0), 1, CV_AA);

//        if (vlines2.size() == 21 && hlines2.size() == 21) {
/*
          Vec2f prev_hline = Vec2f(0,0);

          for(size_t j=0;j<vlines2.size();j++) {
            Vec2f lp = vlines2[j];

            float ang = lp[1];
            float r = lp[0];

            ang = ang/CV_PI*180;

            if (ang < 0) ang+=180;
            if (ang >= 180) ang-=180;

            if (prev_hline[0] == 0) {
              printf("ang: %1.f, r=%1.f\n", ang, r);
            } else {
              printf("ang: %1.f, r=%1.f, dr=%1.f\n", ang, r, abs(prev_hline[0]-r));
            }
            prev_hline = lp;
            polarLine(dst, lp);
          }

          prev_hline = Vec2f(0,0);

          for (size_t j=0;j<hlines2.size();j++) {
            Vec2f lp = hlines2[j];

            float ang = lp[1];
            float r = lp[0];

            ang = ang/CV_PI*180;

            if (ang < 0) ang+=180;
            if (ang >= 180) ang-=180;

            if (prev_hline[0] == 0) {
              printf("ang: %1.f, r=%1.f\n", ang, r);
            } else {
              printf("ang: %1.f, r=%1.f, dr=%1.f\n", ang, r, abs(prev_hline[0]-r));
            }
            prev_hline = lp;

            polarLine(dst, lp);

          }
//        }
 */
/*
        if (output.empty()) {
          output.create(frame.size(), frame.type());
        }

        int x,y;
        for(y=0;y < frame_height;y++) {
          for(x=0;x < frame_width;x++) {
            cv::Point pt(x,y);

            cv::Vec3b ct_b = back.at<cv::Vec3b>(pt);
            cv::Vec3b ct_c = clouds.at<cv::Vec3b>(cv::Point(cx,cy));
            cv::Vec3b ct_f = frame.at<cv::Vec3b>(pt);
            //alpha
            uint8_t ac = fog_alpha.at<uint8_t>(pt);
            uint8_t af = fore_alpha.at<uint8_t>(pt);

            //highlight mask
//            uint8_t hl = pyramid.at<uint8_t>(pt);

//            printf("%lf ",a);

//            if (a>1.0) a=1.0;
//            if (a<0.0) a=0.0;

            cv::Vec3b cl;
*//*
            if (y<y_crop_hist) {
              if (y>=y_crop_hist - 10) {
                float ah = float(y_crop_hist-y)/10.0;
                ac = uint8_t(float(ac)*(ah));
                af = uint8_t((1.0-ah)*af);
              } else {
                af = 0;
              }
            } else {
              ac = 0;
//              if (y<y_crop_hist + 10) {
//                af = uint8_t(255.0 * (float(y-y_crop_hist)/10.0));
//              }

            }
*//*
//            uint8_t m = (ac>af?ac:af);
            cl = (ct_b * (1.0-float(af)/255.0) + ct_f*(float(af)/255.0));
            cl = (cl * (1.0-float(ac)/255.0) + ct_c*(float(ac)/255.0));

            output.at<cv::Vec3b>(pt) = cl;
//            output.at<uint8_t>(pt) = a;

          }
        }
        //move clouds
        cloud_offset+=cloud_delta;


//        cv::add(frame, fore, frame);

        timer_log("time_blend");
*/

        cv::imshow("Frame",dst);
        cv::imshow("Canny",edges);
/*
        cv::imshow("Fore",fore_alpha);
        cv::imshow("Cam",frame);
*/
        timer_log("time_imshow");
/*
        video.write(output);

        if (i==30) {
            update_bg_model = !update_bg_model;
            if(update_bg_model)
                printf("Background update is on\n");
            else
                printf("Background update is off\n");
        }
*/

        char k = (char)cv::waitKey(1);
        if( k == 27 ) break;
/*        if( k == ' ' )
        {
            update_bg_model = !update_bg_model;
            if(update_bg_model)
                printf("Background update is on\n");
            else
                printf("Background update is off\n");
        }
*/
        timer_log("time_total", true);
    }
    return 0;
}




