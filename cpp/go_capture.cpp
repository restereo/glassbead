struct capture_thread_data {
  int ready;
  cv::Mat frame;
  cv::Mat edges;
  cv::Mat gray;
  pthread_mutex_t ready_mutex;
  pthread_cond_t ready_cond;
  
  pthread_mutex_t empty_mutex;
  pthread_cond_t empty_cond;
};

//time-averaging of frames;
const double new_frame_weight = 0.50;

using cv::CLAHE;

void increaseContrast (Mat &src, Mat &dst) {

  // /*
  Ptr<CLAHE> clahe = createCLAHE();
  clahe->setClipLimit(1);

  clahe->apply(src, dst);
  //
  // threshold(*src, *dst, 50, 255, 0);
}


void *captureThreadMain(void *t_data) {

    struct capture_thread_data *td;
    td = (struct capture_thread_data *)t_data;

    cv::Mat frame;
    cv::Mat frame_tm;
    cv::Mat src_gray;
    cv::Mat edges;
  
    cv::VideoCapture cap(0);
//    cap.set(CV_CAP_PROP_FRAME_WIDTH,1280);
    // cap.set(CV_CAP_PROP_FRAME_WIDTH,640);
//    cap.set(CV_CAP_PRP_FRAME_HEIGHT, 720);
    // cap.set(CV_CAP_PROP_FRAME_HEIGHT, 480);


//    cv::VideoCapture cap("./video_goban_1280x960_1.avi");

    int frame_width=    cap.get(CV_CAP_PROP_FRAME_WIDTH);
    int frame_height=   cap.get(CV_CAP_PROP_FRAME_HEIGHT);

    printf("cap init: %dx%d\n", frame_width, frame_height);

    struct timeval st;

    for(;;) {
        timer_start(&st);

        cap >> frame;

        if (frame.empty()) {
          printf("Capture: got empty frame! bailing out!\n\n");
          break;
        }

        timer_log(&st, "cap: time_capture");

        if (frame_tm.empty()) {
          frame_tm.create( frame.size(), frame.type() );
          frame.copyTo(frame_tm);
        } else {
          addWeighted(frame, new_frame_weight, frame_tm, (1.0-new_frame_weight), 0, frame_tm);
        }

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

        timer_log(&st, "cap: time_sobel");

        //for opencv/hough threshold is required
        cv::threshold(edges, edges, 60, 255, 0);

        int erosion_size = 3;
        Mat element =getStructuringElement(MORPH_ELLIPSE,
          Size(2*erosion_size+1,2*erosion_size+1),
          Point(erosion_size,erosion_size));
        cv:morphologyEx(edges, edges, MORPH_CLOSE, element);

            erosion_size = 1;
            element =getStructuringElement(MORPH_ELLIPSE,
          Size(2*erosion_size+1,2*erosion_size+1),
          Point(erosion_size,erosion_size));

        cv::morphologyEx(edges, edges, MORPH_OPEN, element);

        //increateContrast
        increaseContrast( src_gray, src_gray);
        // equalizeHist(src_gray, eq_img);
        // eq_img = src_gray;

        timer_log(&st, "cap: time_prepare_image");
        

        pthread_mutex_lock(&(td->empty_mutex));
        while (td->ready == 1) 
          pthread_cond_wait(&(td->empty_cond), &(td->empty_mutex));
        pthread_mutex_unlock(&(td->empty_mutex));

        timer_log(&st, "cap: time_empty_mutex_lock_wait");

        pthread_mutex_lock(&(td->ready_mutex));
        
        timer_log(&st, "cap: time_ready_mutex_lock");
        
        if ((td->frame).empty()) {
          (td->frame).create(frame.size(), frame.type());
        }
        
        if ((td->edges).empty()) {
          (td->edges).create(edges.size(), edges.type()); 
        }  

        if ((td->gray).empty()) {
          (td->gray).create(src_gray.size(), src_gray.type()); 
        }  
        
        frame.copyTo(td->frame);
        edges.copyTo(td->edges);
        src_gray.copyTo(td->gray);

        td->ready = 1;
        pthread_cond_signal(&(td->ready_cond));
        pthread_mutex_unlock(&(td->ready_mutex));
    }

    td->ready = 0;
    printf("capture thread exit!\n");
    pthread_exit(NULL);
}
