/* Timer and loggin */
void timer_start(struct timeval *st) {
  gettimeofday(st, NULL);
}

void timer_log(struct timeval *st, const char *msg, bool show_fps=false) {
  struct timeval now;
  gettimeofday(&now, NULL);
  double tm = double(now.tv_sec - st->tv_sec)+double(now.tv_usec - st->tv_usec)/1000000.0;
  if (show_fps) {
    printf("%s: %0.3lf, fps: %0.1lf\n\n", msg, tm, 1.0/tm);
  } else {
    printf("%s: %0.3lf\n", msg, tm);
  }
}

/* polar coords */
Vec2f toPolar(Vec4i l) {
    float ang = -atan2((float)(l[2]-l[0]), (float)(l[3]-l[1]));
    float r = ((float)(l[1])*sin((double)ang) + (float)l[0]*cos((double)ang));
    return Vec2f(r, ang);
}

void polarLine(Mat dst, Vec2f lp, Scalar cl) {
          //draw
          double a = cos(lp[1]);
          double b = sin(lp[1]);

          double x0 = a*lp[0], y0 = b*lp[0];
          Point pt1(cvRound(x0 + 2000*(-b)), cvRound(y0 + 2000*(a)));
          Point pt2(cvRound(x0 - 2000*(-b)), cvRound(y0 - 2000*(a)));

          line(dst, pt1, pt2, cl, 1, 8 );
}

void polarLine(Mat dst, Vec2f lp) {
  polarLine(dst,lp,Scalar(0,255,0));
}

Point polarIntersect(Vec2f &a, Vec2f &b) {

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

/* Hough lines processing */

/*
bool polarSortByR(Vec4i *a, Vec4i *b) {
  Vec2f ap = toPolar(*a);
  Vec2f bp = toPolar(*b);
//  printf("ar: %1.f, br: %1.f\n", ap[0], bp[0]);
  return ap[0] > bp[0];
}
*/
bool polarSortByR(Vec2f a, Vec2f b) {
  return a[0] > b[0];
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

        float _ang = 180.0 * (float)maxA / (float)numangle;

        printf("maxA: %d / %d -> %2.2f\n", maxA, numangle, _ang);

        return _ang;
}

float findBestStep(vector<Vec2f> hlines2, vector<Vec2f> vlines2, float threshold) {

        int maxdr = 50;
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

        sort(hlines.begin(), hlines.end(), polarSortByR);

        return hlines;
}

vector<Vec2f> filterHLines(cv::vector<Vec2f> hlines, float merge_dist) {
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

/* dumpLines: some debug */
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

