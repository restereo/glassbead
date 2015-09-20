#include<opencv2/opencv.hpp>
#include<iostream>
#include<vector>
#include<stdio.h>
#include <sys/time.h>
#include <sys/types.h>

#include <stdlib.h>
#include <netdb.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <unistd.h>
#include <pthread.h>

using namespace std;
using namespace cv;

#include "go_funcs.cpp"
#include "go_capture.cpp"



#define DIST_BAD -1
#define DIST_TOOCLOSE 0
#define DIST_ONE 1
#define DIST_TWO 2
#define DIST_THREE 3
#define DIST_TOO_LONG 9

int checkDist(Vec2f a, Vec2f b, float f_dr, float threshold) {
  float dr = fabs(a[0] - b[0]);

  if (dr < f_dr-threshold) {
      return DIST_TOOCLOSE;
  } else if (fabs(dr-f_dr) < threshold) {
      return DIST_ONE;
  } else if ((fabs(dr - 2.0*f_dr) < 1.8*threshold) || ((fabs(dr - 2.0*f_dr) < 3.0*threshold)&&(dr>2.0*f_dr))) {
      return DIST_TWO;
  } else if ((fabs(dr - 3.0*f_dr) < 3.5*threshold) || ((fabs(dr - 3.0*f_dr) < 4.0*threshold)&&(dr>3.0*f_dr))) {
      return DIST_THREE;
  }
  if (dr > 3*f_dr) return DIST_TOO_LONG;
  return DIST_BAD;
}

#define STATUS_REMOVE 0
#define STATUS_OK 1
#define STATUS_ADDONE 2
#define STATUS_ADDTWO 3

void processLinesStatuses(vector<Vec2f> &hlines2, AutoBuffer<int> &status, vector<Vec2f> &hlines3) {
  //process lines
  for(int j=0;j<hlines2.size();j++) {
    if (status[j] == STATUS_REMOVE) continue;
    if (status[j] == STATUS_OK) {
      hlines3.push_back(hlines2[j]);
    }
    if (status[j] == STATUS_ADDONE) {
      if (j==0) {
        printf("ERROR: adding -1-st line\n");
      } else {
        for(int k=j-1;k>=0;k--) {
          printf("back: one@ j:%2d k:%2d st[j]:%d st[k]:%d\n",j,k,status[j],status[k]);
          if (status[k]!=STATUS_REMOVE) {
            Vec2f lp = (hlines2[j]+hlines2[k])/2.0;
            hlines3.push_back(lp);
            hlines3.push_back(hlines2[j]);
            break; //inner loop
          }
        }

      }
    }
    if (status[j] == STATUS_ADDTWO) {
        for(int k=j-1;k>=0;k--) {
          printf("back: two@ j:%2d k:%2d st[j]:%d st[k]:%d\n",j,k,status[j],status[k]);
          if (status[k]!=STATUS_REMOVE) {
            Vec2f lp1 = (hlines2[j]+2.0*hlines2[k])/3.0;
            Vec2f lp2 = (2.0*hlines2[j]+hlines2[k])/3.0;
            hlines3.push_back(lp1);
            hlines3.push_back(lp2);
            hlines3.push_back(hlines2[j]);
            break; //inner loop
          }
        }

    }
  }
}

void filterGridLines(vector<Vec2f> &hlines2, float f_dr, float threshold, vector<Vec2f> &hlines3) {

  AutoBuffer<int> status(hlines2.size());

  status[0] = STATUS_REMOVE;

  //pass1 for DIST_ONE
  for(size_t j=1;j<hlines2.size();j++) {
      status[j] = STATUS_REMOVE;

      int q = checkDist(hlines2[j-1],hlines2[j],f_dr,threshold);
      if (q == DIST_ONE) {
        status[j-1] = STATUS_OK;
        status[j] = STATUS_OK;
        continue;
      }
  }
  //forward patching
  for(size_t j=1;j<hlines2.size();j++) {
//    printf("fp: j=%d\n",j);
    if ((status[j-1] == 1) && (status[j] == 1)) continue;
    if ((status[j-1] == 1) && (status[j] == 0)) {
      for(size_t k=j;k<hlines2.size(); k++) {
        if (status[k]>0) break; //dont go past good lines
        float dr = fabs(hlines2[k][0] - hlines2[j-1][0]);
        if (dr < f_dr) continue; //skip a line
        int q = checkDist(hlines2[k],hlines2[j-1], f_dr, threshold);
//        printf("fj:%2d k:%2d st[j-1]: %d st[k]:%d dr(j-k):%2.2f\n",j,k,status[j-1],status[k], dr);
          if (q==DIST_TWO) {
            printf("fj: one@%d-%d [%d]\n", j-1, k, status[k]);
            status[k] = STATUS_ADDONE;
            j=k;
            break;
          }
          if (q==DIST_THREE) {
            printf("fj: two@%d-%d [%d]\n", j-1, k, status[k]);
            status[k] = STATUS_ADDTWO;
            j=k;
            break;
          }
          if (q==DIST_TOO_LONG) {
            break;
          }
      }
    }
  }

    //backward patching
  for(int j=hlines2.size();j>1;j--) {
//    printf("bp: j=%d\n",j);
    if ((status[j] == 1) && (status[j-1] == 0)) {
      for(int k=j-1;k>=0; k--) {
        if (status[k]>0) break; //dont go past good lines
        float dr = fabs(hlines2[k][0] - hlines2[j][0]);
          if (dr < f_dr) continue; //skip a line
          int q = checkDist(hlines2[k],hlines2[j], f_dr, threshold);
//          printf("bj:%2d k:%2d st[j]: %d st[k]:%d dr(j-k):%2.2f\n",j,k,status[j],status[k], dr);
          if (q==DIST_TWO) {
            printf("bj: one@%d-%d [%d]\n", k, j, status[j]);
            status[j] = STATUS_ADDONE;
            if (status[k]==STATUS_REMOVE) status[k]=STATUS_OK;
            if (k>1) j=k;
            break;
          }
          if (q==DIST_THREE) {
            printf("bj: two@%d-%d [%d]\n", k, j, status[j]);
            status[j] = STATUS_ADDTWO;
            if (status[k]==STATUS_REMOVE) status[k]=STATUS_OK;
            if (k>1) j=k;
            break;
          }
          if (q==DIST_TOO_LONG) {
            break;
          }
      }
    }

  }

  int good_c = 0;
  for(size_t j=0;j<hlines2.size();j++) {
    good_c+=status[j];
  }

  if(good_c == 20) {
    if (status[0]==STATUS_REMOVE) {
      status[hlines2.size()-1] = STATUS_REMOVE;
    } else if (status[hlines2.size()-1] == STATUS_REMOVE) {
      status[0]=STATUS_REMOVE;
    }
  } else if (good_c == 21) {

    status[0] = STATUS_REMOVE;
    status[hlines2.size()-1] = STATUS_REMOVE;
  }

  processLinesStatuses(hlines2, status, hlines3);



  //debug code
  good_c = hlines3.size();

  printf("1d[%d/%d]: ",good_c,hlines2.size());
  for(size_t j=0;j<hlines2.size();j++) {
    if (j>0) {
      float dr = fabs(hlines2[j-1][0] - hlines2[j][0]);
//      if (status[j-1]==STATUS_REMOVE || (status[j]!=STATUS_REMOVE && status[j-1]==STATUS_REMOVE) || (status[j] == STATUS_REMOVE && status[j-1]!=STATUS_REMOVE))
      printf(" [%2.2f] ", dr);
    } else {
//      float dr = fabs(hlines2[0][0] - hlines2[1][0]);
//      if (status[j]==0) printf("[%2.2f] ", dr);
    }
    printf("%d",status[j]);
  }
  printf("\n");

}


void makeSomeGrid(vector<Vec2f> &hlines2, vector<Vec2f> &vlines2, float f_dr, Mat dst, float threshold, vector<Vec2f>& hlines3, vector<Vec2f>& vlines3) {

  filterGridLines(hlines2, f_dr, threshold, hlines3);
  filterGridLines(vlines2, f_dr, threshold, vlines3);

  for(size_t j = 0; j<hlines3.size();j++) polarLine(dst, hlines3[j], Scalar(0,0,255));
  for(size_t j = 0; j<vlines3.size();j++) polarLine(dst, vlines3[j], Scalar(0,0,255));


/*
        if (dr < f_dr-2*threshold) {
          printf("too close\n");
          polarLine(dst, hlines2[j], Scalar(255,0,0));
        } else if (fabs(dr-f_dr) < threshold) {
          //looks good
          hlines3.push_back(hlines2[j]);
        } else if (fabs(dr - 2*f_dr) < threshold) {
          //we miss a line
          printf("missing line_%d_",j);

//          polarLine(dst, (hlines2[j]+hlines2[j-1])/2, Scalar(255,0,0));
//          hlines3.push_back((hlines2[j]+hlines2[j-1])/2);
          hlines3.push_back(hlines2[j]);

        } else if (fabs(dr - 3*f_dr) < threshold) {
          printf("missing2lines_%d_",j);
//          hlines3.push_back((hlines2[j]+2*hlines2[j-1])/3.0);
//          hlines3.push_back((2*hlines2[j]+hlines2[j-1])/3.0);

          polarLine(dst, (hlines2[j]+2.0*hlines2[j-1])/3.0, Scalar(255,0,0));
          polarLine(dst, (2.0*hlines2[j]+hlines2[j-1])/3.0, Scalar(255,0,0));


          hlines3.push_back(hlines2[j]);
        }
      }
      prev_r = hlines2[j][0];
  */



//  printf("somelines: h: %d -> %d\n", hlines2.size(), hlines3.size());

//  hlines2=hlines3;

        if ((hlines3.size() == 19) && (vlines3.size() == 19)) {

          printf("render rect\n");

          Point pt1 = polarIntersect(hlines3[0], vlines3[0]);
          Point pt2 = polarIntersect(hlines3[0], vlines3[vlines3.size()-1]);
          Point pt3 = polarIntersect(hlines3[hlines3.size()-1], vlines3[0]);
          Point pt4 = polarIntersect(hlines3[hlines3.size()-1], vlines3[vlines3.size()-1]);

          line(dst, pt1, pt2, Scalar(0,0,255), 3, 8 );
          line(dst, pt2, pt4, Scalar(0,0,255), 3, 8 );
          line(dst, pt3, pt4, Scalar(0,0,255), 3, 8 );
          line(dst, pt3, pt1, Scalar(0,0,255), 3, 8 );
        }

}





inline int _min(int a, int b) { return ((a>b)?b:a); }
inline int _max(int a, int b) { return ((a>b)?a:b); }

char getPixelAtPoint(Mat &eq_img, Point &p, int delta) {

  // printf("getPixelAtPoint: %d %d", p.x, p.y);

  //histo params
  const int divider = 4;
  const int hst_len = 256/divider;

  AutoBuffer<int> _hst(hst_len);
  int *hst = _hst;

  memset(hst, 0, hst_len * sizeof(int));

  //printf("@rect: [%d %d] [%d %d]\n", _max(0,p.y-delta),_min(eq_img.rows, p.y+delta),_max(0,p.x-delta),_min(eq_img.cols, p.x+delta));

  for (int i = max(0,p.y-delta); i < min(eq_img.rows, p.y+delta+1); i++)
  {
    for (int j = max(0,p.x-delta); j < min(eq_img.cols, p.x+delta+1); j++)
    {
      if ((i>=eq_img.rows) || (j>=eq_img.cols)) {
        printf("ERR: i=%d, j=%d\n",i,j);
      }
      uint8_t __i = eq_img.at<uchar>(i, j); //y x
      hst[(__i/divider)]++;
    }
  }


  AutoBuffer<int> _hstb(hst_len);
  int *hstb = _hstb;
  memset(hstb,0,hst_len*sizeof(int));
  for(int i=0;i<hst_len-1;i++) {
    hstb[i]=hst[i]+hst[i+1];
  }

  //process histo
  int maxHst_val =0;
  int maxHst_pos = 0;
  for(size_t l=0;l<hst_len;l++) {
    if (hstb[l]>maxHst_val) {
      maxHst_val = hstb[l];
      maxHst_pos = l;
    }
  }

  char resB = '*';

  if (maxHst_val > 60) { //65 misses some bricks
    if (maxHst_pos > 180/divider) {
      resB = 'W';
    } else if (maxHst_pos < 90/divider) {
      resB = 'B';
    }
  }

  //move all debug to the end/conditional
  if (0 && resB!='*' ) {
    
    printf("Histo @(%d,%d):\n",p.x,p.y);
    for(size_t l=0;l<hst_len;l++) {
      printf("%2d ",hst[l]);
      if (l%32==31) printf("\n");    
    }

    printf("HistoB @(%d,%d):\n",p.x,p.y);
    for(size_t l=0;l<hst_len;l++) {
      printf("%2d ",hstb[l]);
      if (l%32==31) printf("\n");    
    }

    printf("maxHst: val=%d pos=%d max brighness => %d\n", maxHst_val, maxHst_pos, maxHst_pos*divider);

    // printf("getPixelAtPoint: %c min:%d max:%d val:%d disp:%d\n", resB, int_hist[0], int_hist[int_hist.size()-1],_i,disp);
  }
  
  return resB;

}

/*!
 * На вход нужны картинка и решетка
 */
vector <vector <char> > getBoard (Mat img, vector<Vec2f> &hlines, vector<Vec2f> &vlines) {

  // assert(hlines.size(), 21);
  // assert(vlines.size(), 21);

  // char[][] board= new char[19][19];
  vector< vector<char> > board;

  board.resize(20, vector<char>(19, '*'));
  Point p;

  for (int i = 0; i < hlines.size() ; i++) {

    for (int j = 0; j < vlines.size() ; j++) {

      // printf("hline: %2.f %2.f\n", hlines[i][0], hlines[i][1]);
      // printf("vline: %2.f %2.f\n", vlines[j][0], vlines[j][1]);
      // printf("got lines\n");

      p = polarIntersect(hlines[i], vlines[j]);

      // printf("point: %d %d \n", p.x, p.y);


      char c = getPixelAtPoint(img, p, 8);

      // printf("%c\n", c);

      board[i][j] = c;

    }
    // board.push_back(_line);
  }

  return board;

}


FILE * _sock;

char _addr[] = "192.168.12.11\0";

FILE * initSocket(int argc, char *argv[]) {

  char *addr_str = &(_addr[0]);

  if (argc!=2) {
    printf("Warn: usage %s <hostname_of_server>\nUsing default of: %s\n", argv[0], _addr);
  } else if (argc==2) {
    addr_str=argv[1];
  }

  printf("Connect: %s\n", addr_str);

  struct sockaddr_in addr;
  struct hostent *server;

  memset(&addr, 0, sizeof(addr));

  addr.sin_family = AF_INET;
  addr.sin_port = htons(7777);

  int sock_fd = socket(AF_INET, SOCK_STREAM, 0);

  if (sock_fd<0) {
    printf("ERROR opening socket\n");
    exit(1);
  }

  server = gethostbyname(addr_str);

  if (server == NULL) {
    printf("GetHostByName: no such host %s\n",addr_str);
    exit(1);
  }

  // inet_pton(AF_INET, "192.168.12.19", &addr.sin_addr.s_addr);

  bcopy((char *)server->h_addr, (char *)&addr.sin_addr.s_addr, server->h_length);

  int e = connect(sock_fd, (struct sockaddr*) &addr, sizeof(addr));

  if (e < 0) {
    printf("\n\nError connecting to %s: %d\n\n", addr_str, e);
     exit(1);
  }

  FILE *fd = fdopen(sock_fd, "w");
  if (fd == NULL) {
    printf("Could not fdopen socket\n");
    exit(1);
  }
  return fd;
}

char _empty_line[] = "\n\n";

void sendToSocket(FILE* fd,vector<vector<char> >& buf) {

  int _s = buf[0].size();

  for (int i = 0; i < 19; i++)
  {
    for (int j = 0; j < 19; j++)
    {
      fputc(buf[i][j], fd);
    }
    // fprintf(fd,"blablabla\n");
  }

  int t = fwrite(_empty_line, 1, 2, fd);
  fflush(fd);
}





int main(int argc, char *argv[])
{
    cv::Mat dst;
    cv::Mat eq_img;
    cv::Mat edges;
    cv::Mat edges_resize;

    FILE* _sock = initSocket(argc, argv);

    //init capture thread
    
    pthread_t capture_thread;
    struct capture_thread_data td;
    td.ready = 0;

    //init mutex
    pthread_mutex_init(&(td.ready_mutex), NULL);
    pthread_cond_init(&(td.ready_cond), NULL);

    pthread_mutex_init(&(td.empty_mutex), NULL);
    pthread_cond_init(&(td.empty_cond), NULL);

    //start after GUI init
    int rc = pthread_create(&capture_thread, NULL, captureThreadMain, (void*)&td);
    if (rc) {
      printf("Could not create capture thread!\n");
      exit(1);
    }

    int good_frames = 0;
    int total_frames = 0;

//    cv::VideoWriter video("mog.avi",CV_FOURCC('M','J','P','G'),12, cv::Size(frame_width,frame_height),true);

    cv::namedWindow("Frame");
    cv::namedWindow("eq_img");

    cv::moveWindow("Frame", 0,0);
    cv::resizeWindow("Frame", 640, 480);
    cv::moveWindow("eq_img", 640,0);
    cv::resizeWindow("eq_img", 640, 480);
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

*/

    int lowTreshold = 25;
    float ratio = 3;
    int kernel_size = 3;

    float f_dr_avg = 0;

    int i=0;


    struct timeval st;

    
    for(;;i++)
    {
        timer_start(&st);


        //get a frame from capture thread
        pthread_mutex_lock(&(td.ready_mutex));
        while (td.ready==0) 
          pthread_cond_wait(&(td.ready_cond), &(td.ready_mutex));
        

        timer_log(&st, "proc: wait for a frame");
    
        if (dst.empty()) {
          dst.create( td.frame.size(), td.frame.type() );
        }
        if (eq_img.empty()) {
          eq_img.create( td.gray.size(), td.gray.type() );
        }
        if (eq_img.empty()) {
          edges.create( td.edges.size(), td.edges.type() );
        }
        td.frame.copyTo(dst);
        td.gray.copyTo(eq_img);
        td.edges.copyTo(edges);
        
        //replace with copy if required
        cv::resize(td.edges, edges_resize, Size(640,480));

        total_frames++;

        td.ready = 0;
        //unlock
        pthread_mutex_unlock(&(td.ready_mutex));


        pthread_cond_signal(&(td.empty_cond));


        timer_log(&st, "proc: get frame");

//2: hough

        int maxangle = 360; //360 => 0.5 degree angle step
        double rho=1.0;
        double theta = CV_PI/(float)maxangle;


        //use smaller
        Mat new_edges;
        cv::resize(edges, new_edges, Size(edges.cols/2,edges.rows/2));
        
        int threshold = 120/2;
        vector<Vec4i> lines;  /* minLineLength, maxLineMissingPart */
        HoughLinesP(new_edges, lines, rho, theta, threshold, 270/2, 40/2);
        printf("hough.lines: %d\n", lines.size());


        timer_log(&st, "proc: hough & unlock");


        //fix scaled coords
        for(size_t j=0; j<lines.size();j++) {
          lines[j][0] = 2.0*lines[j][0];
          lines[j][1] = 2.0*lines[j][1];
          lines[j][2] = 2.0*lines[j][2];
          lines[j][3] = 2.0*lines[j][3];
        }

/*
        for(size_t j=0; j<lines.size(); j++) {
          Vec4i l = lines[j];
          line(dst, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255,0,0), 1, 8);
        }
*/
        
        /* fixme: only supports 180 buckets */
        float maxA = findBestAngle(lines, maxangle); //180/2 degree values

        vector<Vec2f> hlines = findHLines(lines, 3.0, maxA);
        vector<Vec2f> vlines = findHLines(lines, 3.0, maxA+90);

//        vector<Vec2f> hlines2 = approxHLines(hlines, 0.3);
        printf("lines: h.size=%d v.size=%d\n", hlines.size(), vlines.size());

        float merge_dist = 8.0;
        vector<Vec2f> hlines2 = filterHLines(hlines, merge_dist);
        vector<Vec2f> vlines2 = filterHLines(vlines, merge_dist);

        printf("lines2: h.size=%d v.size=%d\n", hlines2.size(), vlines2.size());

        for(size_t j=0; j<hlines2.size(); j++) polarLine(dst, hlines2[j]);
        for(size_t j=0; j<vlines2.size(); j++) polarLine(dst, vlines2[j]);

/*
        dumpLines(vlines);
*/


        float f_dr = findBestStep(hlines2, vlines2, 1.5);
        if (f_dr > 10) {
          if (f_dr_avg == 0) {
            f_dr_avg = f_dr;
          } else {
            f_dr_avg = (0.1*f_dr + 0.9*f_dr_avg);
          }
        }
/*
        vector<Vec2f> hlines3 = filterLinesByDR(hlines2, f_dr, 5.5);
        vector<Vec2f> vlines3 = filterLinesByDR(vlines2, f_dr, 5.5);
*/


//        printf("hlines3.size = %d x %d\n",hlines3.size(), vlines3.size());
//        for(size_t j=0; j<hlines3.size(); j++) polarLine(dst, hlines3[j]);
//        for(size_t j=0; j<vlines3.size(); j++) polarLine(dst, vlines3[j]);

        vector <Vec2f> hlines3, vlines3;


        float SomeThresh = 2.5*2.5;
        printf("f_dr_avg: %2.2f, thresh: %2.2f\n",f_dr_avg, SomeThresh);

        makeSomeGrid(hlines2,vlines2, f_dr_avg, dst, SomeThresh, hlines3, vlines3);

        timer_log(&st, "time_line_filter");



        if ((hlines3.size() == 19) && (vlines3.size() == 19)) {
          good_frames++;
        // char board[19][19];

          vector <vector <char> > board = getBoard(eq_img, hlines3, vlines3);

          printf("\n");
          for (int i = 0; i < 19; i++)
          {
            for (int j = 0; j < 19;j++)
            {
              printf("%c", board[i][j]);
            }
            printf("\n");
          }
          
           sendToSocket(_sock, board);

        }

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

        timer_log(&st, "time_blend");
*/
        Mat dst_resize;
        cv::resize(dst, dst_resize, Size(640,480));
        
        cv::imshow("Frame",dst_resize);
//        cv::imshow("Canny",edges);
        cv::imshow("eq_img",edges_resize);
/*
        cv::imshow("Cam",frame);
*/
        timer_log(&st, "time_imshow");
        printf("frames good/total: %6d/%6d %2.2f\n", good_frames, total_frames, (100.0*(float)good_frames/(float)total_frames));
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
        if( k == ' ' )
        {
          fflush(stdout);
          while(true) {
            k = (char)cv::waitKey(100);
            if (k == ' ') break;
          }
        }
        timer_log(&st, "time_total", true);
    }
    return 0;
}



