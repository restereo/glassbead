#define TH_STATUS_EMPTY 0
#define TH_STATUS_HAVE_EDGES 1
#define TH_STATUS_HAVE_LINES 2

struct hough_thread_data {
  int status;
  
  cv::Mat edges;
  vector<Vec4i> lines;

  pthread_mutex_t edges_mutex;
  pthread_cond_t edges_cond;
  
  pthread_mutex_t lines_mutex;
  pthread_cond_t lines_cond;
};

const int maxangle = 360; //360 => 0.5 degree angle step
const double rho=1.0;
const double theta = CV_PI/(float)maxangle;

using cv::CLAHE;

void increaseContrast (Mat &src, Mat &dst) {

  // /*
  Ptr<CLAHE> clahe = createCLAHE();
  clahe->setClipLimit(1);

  clahe->apply(src, dst);
  //
  // threshold(*src, *dst, 50, 255, 0);
}


void *houghThreadMain(void *t_data) {

    struct hough_thread_data *td;
    td = (struct hough_thread_data *)t_data;

    cv::Mat edges;
  
    struct timeval st;

    for(;;) {
        timer_start(&st);
        
        printf("proc: waiting for edges: status=%d\n",td->status);
        
        pthread_mutex_lock(&(td->edges_mutex));
        while ( (td->status & TH_STATUS_HAVE_EDGES) == 0 ) {
          int res = pthread_cond_wait(&(td->edges_cond), &(td->edges_mutex));
        }

        //get frame
        (td->edges).copyTo(edges);  
        td->status = td->status & ~TH_STATUS_HAVE_EDGES;

        pthread_mutex_unlock(&(td->edges_mutex));

        if (edges.empty()) {
          printf("proc: got empty frame! bailing out!\n\n");
          break;
        }

        timer_log(&st, "proc: time_get_frame");

//2: hough



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

        pthread_mutex_lock(&(td->lines_mutex));
        
        timer_log(&st, "proc: time_lines_mutex_lock");
        
        (td->lines) = lines;
        td->status |= TH_STATUS_HAVE_LINES;
        
        pthread_mutex_unlock(&(td->lines_mutex));
        pthread_cond_signal(&(td->lines_cond));
    }

    td->status = 0;
    printf("capture thread exit!\n");
    pthread_exit(NULL);
}
