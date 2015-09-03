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
/*
bool waytosort(Vec4i *a, Vec4i *b) {
  Vec2f ap = toPolar(*a);
  Vec2f bp = toPolar(*b);
//  printf("ar: %1.f, br: %1.f\n", ap[0], bp[0]);
  return ap[0] > bp[0];
}
*/
bool waytosort(Vec2f a, Vec2f b) {
  return a[0] > b[0];
}
void polarLine(Mat dst, Vec2f lp, Scalar cl) {
          //draw
          double a = cos(lp[1]);
          double b = sin(lp[1]);

          double x0 = a*lp[0], y0 = b*lp[0];
          Point pt1(cvRound(x0 + 1000*(-b)), cvRound(y0 + 1000*(a)));
          Point pt2(cvRound(x0 - 1000*(-b)), cvRound(y0 - 1000*(a)));

          line(dst, pt1, pt2, cl, 1, 8 );

}
void polarLine(Mat dst, Vec2f lp) {
  polarLine(dst,lp,Scalar(0,255,0));
}

Point intersect(Vec2f &a, Vec2f &b) {

  // printf("line h: %1.f %1.f\n", a[0], a[1]);

   float x1 = a[0]*cos(a[1]);
   float y1 = a[0]*sin(a[1]);

   float x2 = b[0]*cos(b[1]);
   float y2 = b[0]*sin(b[1]);

   float t2 = ((y2-y1)*sin(a[1]) + (x2-x1)*cos(a[1])) / (cos(b[1])*sin(a[1]) - sin(b[1])*cos(a[1]));

   float x = x2 + t2*sin(b[1]);
   float y = y2 - t2*cos(b[1]);

   // printf("(x,y) = (%1.f, %1.f)\n", x, y);

   Point p = Point(floor(x + 0.5), floor(y  + 0.5));

   return p;
}

/*
int solve(hlines, path) {

}
*/

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
#define STATUS_ADDTHREE 3

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
    printf("fp: j=%d\n",j);
    if ((status[j-1] == 1) && (status[j] == 1)) continue;
    if ((status[j-1] == 1) && (status[j] == 0)) {
      for(size_t k=j;k<hlines2.size(); k++) {
        if (status[k]>0) break; //dont go past good lines 
        float dr = fabs(hlines2[k][0] - hlines2[j-1][0]);
        if (dr < f_dr) continue; //skip a line
        int q = checkDist(hlines2[k],hlines2[j-1], f_dr, threshold);
        printf("fj:%2d k:%2d st[j-1]: %d st[k]:%d dr(j-k):%2.2f\n",j,k,status[j-1],status[k], dr);
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
    printf("bp: j=%d\n",j);
    if ((status[j] == 1) && (status[j-1] == 0)) {
      for(int k=j-1;k>=0; k--) {
        if (status[k]>0) break; //dont go past good lines 
        float dr = fabs(hlines2[k][0] - hlines2[j][0]);
          if (dr < f_dr) continue; //skip a line
          int q = checkDist(hlines2[k],hlines2[j], f_dr, threshold);
          printf("bj:%2d k:%2d st[j]: %d st[k]:%d dr(j-k):%2.2f\n",j,k,status[j],status[k], dr);
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
  
  processLinesStatuses(hlines2, status, hlines3);


  //hmm... magick dirty fixes
  
  if (hlines3.size() > 19) {
    //remove some lines;
    int dirty_start = 0;
    for(int i=1;i<min(4,(int)hlines2.size()); i++) {
      if (status[i] >= STATUS_ADDONE) {
          dirty_start = i;
          break;
      }
    }
    
    printf("dirty_start: %d\n", dirty_start);
    
    if (dirty_start) {
                    
    }
    
    
  }

  //debug code
  int good_c = hlines3.size();

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


void makeSomeGrid(vector<Vec2f> &hlines2, vector<Vec2f> &vlines2, float f_dr, Mat dst, float threshold) {

  vector<Vec2f> hlines3, vlines3;
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

          Point pt1 = intersect(hlines3[0], vlines3[0]);
          Point pt2 = intersect(hlines3[0], vlines3[vlines3.size()-1]);
          Point pt3 = intersect(hlines3[hlines3.size()-1], vlines3[0]);
          Point pt4 = intersect(hlines3[hlines3.size()-1], vlines3[vlines3.size()-1]);

          line(dst, pt1, pt2, Scalar(0,0,255), 3, 8 );
          line(dst, pt2, pt4, Scalar(0,0,255), 3, 8 );
          line(dst, pt3, pt4, Scalar(0,0,255), 3, 8 );
          line(dst, pt3, pt1, Scalar(0,0,255), 3, 8 );
        }

}

//fixed support for smaller angle step
float findBestAngle(vector<Vec4i> lines, int numangle) {

        AutoBuffer<int> _counts(numangle);
        int *counts = _counts;
        memset(counts, 0, numangle * sizeof(int));

        for( size_t j=0; j<lines.size(); j++) {
          Vec4i l = lines[j];
          Vec2f lp = toPolar(l);

          float ang = lp[1]; //(180/numagle)

          if (ang < 0) ang+=CV_PI;
          if (ang >= CV_PI) ang-=CV_PI;

          // 0 to 90 deg
          if (ang >= CV_PI/2.0 ) ang -= CV_PI/2.0;

          int _ang = cvRound(ang/CV_PI*numangle);
          if ((_ang>=0) && (_ang<numangle)) {
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
        printf("maxA: %d / %d\n", maxA, numangle);


        return (180.0 * (float)maxA / (float)numangle);
}

float findBestStep(vector<Vec2f> hlines2, vector<Vec2f> vlines2, float threshold) {

        int maxdr = 30;
        AutoBuffer<int> _dritems(maxdr);
        memset(_dritems, 0, sizeof(int)*(maxdr));

        float prev_r;
        for(size_t j=0;j<vlines2.size();j++) {
          if (j!=0) {
            int dr = cvRound(fabs(prev_r - vlines2[j][0]));
            if (dr<maxdr) {
              _dritems[dr]++;
            }
          }
          prev_r = vlines2[j][0];
        }

        for(size_t j=0;j<hlines2.size();j++) {
          if (j!=0) {
            int dr = cvRound(fabs(prev_r - hlines2[j][0]));
            if (dr<maxdr) {
              _dritems[dr]++;
            }
          }
          prev_r = hlines2[j][0];
        }

        int bestdr_items = 0;
        int bestdr = 0;
        for(size_t j=0; j< maxdr; j++) {
          if (bestdr_items <= _dritems[j]) {
            bestdr_items = _dritems[j];
            bestdr = j;
          }
//          printf("dr=%d, count=%d\n", j, _dritems[j]);
        }

        //average float dr
        float f_dr = 0;
        int   n_dr = 0;

        for(size_t j=0;j<vlines2.size();j++) {
          if (j!=0) {
            float dr = fabs(prev_r - vlines2[j][0]);
            if (fabs(dr - (float)bestdr) < threshold) {
              f_dr+=dr;
              n_dr++;
            }
          }
          prev_r = vlines2[j][0];
        }

        for(size_t j=0;j<hlines2.size();j++) {
          if (j!=0) {
            float dr = fabs(prev_r - hlines2[j][0]);
            if (fabs(dr - (float)bestdr) < threshold) {
              f_dr+=dr;
              n_dr++;
            }
          }
          prev_r = hlines2[j][0];
        }

        if (n_dr == 0) {
            f_dr = 0;
        } else {
            f_dr = f_dr / (float)n_dr;
        }
        printf("bestdr: %d %2.5f\n", bestdr, f_dr);


        return f_dr;
}

vector<Vec2f> approxHLines(vector<Vec2f> lines2, float threshold) {
        vector<Vec2f> lines3;

        float sum_r =0, sum_a =0, sum_r2=0, sum_ra=0;

        for(size_t j=0;j<lines2.size();j++) {
            sum_r += lines2[j][0];
            sum_a += lines2[j][1];
            sum_r2+= lines2[j][0]*lines2[j][0];
            sum_ra+= lines2[j][1]*lines2[j][0];
        }
        size_t n = lines2.size();
        float a = ((float)n*sum_ra - sum_r*sum_a) / ((float)n*sum_r2 - sum_r*sum_r);
        float b = (sum_a - a*sum_r) / (float)n;

        printf("approx line polar: ang = %0.6f * r + %0.6f\n", a, b);
        printf("min_r=%1.2f, max_r=%1.2f, diff_a=%1.2f\n", lines2[0][0], lines2[n-1][0], fabs(180*lines2[0][1]/CV_PI - 180*lines2[n-1][1]/CV_PI));

        float da2 = 0;
        float da  = 0;

        for(size_t j=0;j<lines2.size();j++) {
            float af = a*lines2[j][0]+b;
            float da = fabs(lines2[j][1] - af);

            da2 += da*da;
            da  += da;
        }

        float dda = sqrt(da2 - da*da);

        for(size_t j=0;j<lines2.size();j++) {
            float af = a*lines2[j][0]+b;
            float da = fabs(lines2[j][1] - af);
            if (da < dda*threshold) {
              lines3.push_back(lines2[j]);
            }
        }


        printf("lines3: %d -> %d dda=%0.6f\n", lines2.size(), lines3.size(), dda);
        return lines3;
}

vector<Vec2f> filterLinesByDR(vector<Vec2f> lines2, float f_dr, float threshold) {
        vector<Vec2f> lines3;
       //filter lines on best step

        int last_pushed_line;
        float prev_r;

        for(size_t j=0;j<lines2.size();j++) {
          if (j==0) {
            //dunno
            last_pushed_line = -1;
          } else {
            float dr;
            if (last_pushed_line == -1) {
                dr = fabs(prev_r - lines2[j][0]);
            } else {
                dr = fabs(lines2[last_pushed_line][0] - lines2[j][0]);
            }

            if (fabs(f_dr - dr) < threshold) {
              if (last_pushed_line == -1) {
                lines3.push_back(lines2[j-1]);
              }
              lines3.push_back(lines2[j]);
              last_pushed_line = j;
            }
          }
          prev_r = lines2[j][0];
        }

        printf("lines3: %d\n", lines3.size());
        return lines3;
}


vector<Vec2f> findHLines(vector<Vec4i> lines, float ang_threshold, float maxA) {
        vector<Vec2f> hlines;

        for( size_t j=0; j<lines.size(); j++) {
          Vec4i l = lines[j];
          Vec2f lp = toPolar(l);

          float ang = lp[1];
          float r = lp[0];

          ang = ang/CV_PI*180;

          if (ang < 0) ang+=180;

          float a = fabs(maxA - ang);
          float aa= fabs(180 + maxA - ang);
          if (a < ang_threshold || aa <ang_threshold) {
            hlines.push_back( lp );
          }

        }

        sort(hlines.begin(), hlines.end(), waytosort);

        return hlines;
}

vector<Vec2f> filterHLines(vector<Vec2f> hlines, float merge_dist) {
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


          if (fabs(last_lp[0] - lp[0]) <= merge_dist) {
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
              printf("ang: %1.f, r=%1.f, dr=%1.f\n", ang, r, fabs(prev_line[0]-r));
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



char getPixelAtPoint(Mat img, Point p, int delta) {


  // printf("getPixelAtPoint: %d %d", p.x, p.y);

  Scalar intensity = img.at<uchar>(p);

  int _i = intensity.val[0]; // from 0 to 255

  int i_point = _i;

  for (int i = -delta; i < delta; i++)
  {
    for (int j = -delta; j < delta; j++)
    {
      Scalar __i = img.at<uchar>(p);

      // int i = intensity.val[0];
      _i = floor((_i + __i.val[0])/2);
    }
  }

  printf("intensity: %d (%d) ", _i, i_point);

  // Vec3f intensity = img.at<Vec3f>(y, x);
  // float blue = intensity.val[0];
  // float green = intensity.val[1];
  // float red = intensity.val[2];
  // float sum = blue + green + red;

  if (_i > 190 ) {
    return 'W';
  }

  if (_i < 50 ) {
    return 'B';
  }

  return '*';

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

  for (int i = 1; i < hlines.size() - 1; i++) {

    for (int j = 1; j < vlines.size() - 1; j++) {

      // printf("hline: %2.f %2.f\n", hlines[i][0], hlines[i][1]);
      // printf("vline: %2.f %2.f\n", vlines[j][0], vlines[j][1]);
      // printf("got lines\n");

      p = intersect(hlines[i], vlines[j]);

      // printf("point: %d %d \n", p.x, p.y);


      char c = getPixelAtPoint(img, p, 3);

      printf("%c\n", c);

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
    cv::Mat frame;
    cv::Mat src_gray;
    cv::Mat dst, edges;

    cv::Mat output;

    cv::Mat frame_tm;

    _sock = initSocket(argc, argv);

//    cv::VideoCapture cap(1);
//    cap.set(CV_CAP_PROP_FRAME_WIDTH,1280);
//    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 720);


    cv::VideoCapture cap("./video/video_640x480.avi");
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

    float f_dr_avg = 0;


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

//        Scharr( src_gray, edges_x, CV_16S, 1, 0, scale, delta, BORDER_DEFAULT);
//        Scharr( src_gray, edges_y, CV_16S, 0, 1, scale, delta, BORDER_DEFAULT);

        Mat abs_grad_x, abs_grad_y;
        convertScaleAbs( edges_x, abs_grad_x );
        convertScaleAbs( edges_y, abs_grad_y );

        addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, edges);

        //for opencv/hough threshold is required
        cv::threshold(edges, edges, 20, 255, 0);



//2: hough

        int maxangle = 360; //360 => 0.5 degree angle step
        double rho=2.0;
        double theta = CV_PI/(float)maxangle;
        int threshold = 150;

/* //My own greyscale-enabled Hough

        printf("call houghimg\n");
        HoughImg(edges, rho, theta, dst);

// Standart Hough

        vector<Vec2f> lines;
        HoughLines(edges, lines, rho, theta, threshold);
*/

        vector<Vec4i> lines;  /* minLineLength, maxLineMissingPart */
        HoughLinesP(edges, lines, rho, theta, threshold, 170, 30);
        printf("hough.lines: %d\n", lines.size());

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

        float merge_dist = 4.0;
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

        float SomeThresh = 2.5;
        printf("f_dr_avg: %2.2f, thresh: %2.2f\n",f_dr_avg, SomeThresh);
        makeSomeGrid(hlines2,vlines2, f_dr_avg, dst, SomeThresh);



        if ((hlines2.size() == 21) && (vlines2.size() == 21)) {

          // char board[19][19];

          vector <vector <char> > board = getBoard(src_gray, hlines2, vlines2);


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


/*

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



