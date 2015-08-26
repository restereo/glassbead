import processing.video.*;
import gab.opencv.*;
import java.awt.*;

int DONT_INTERSECT = 0;
int COLLINEAR = 1;
int DO_INTERSECT = 2;

float x =0, y=0;

int x1, x2, x3, x4, y1, y2, y3, y4, f_rate, camera_height, camera_width;

Capture cam;
OpenCV opencv;

void setup() {

  f_rate = 30;
  camera_width = 1280;
  camera_height = 960;

  String[] cameras = Capture.list();

  String camera_name = "";

  if (cameras.length == 0) {
    println("There are no cameras available for capture.");
    exit();
  } else {
    for (int i = 0; i < cameras.length; i++) {

      println(cameras[i]);

      String[] m1 = match(cameras[i], "size=1280x960,fps=30");

      if (m1 != null) {
        camera_name = cameras[i];
      }
    }
  }

  if (camera_name == "") {
    camera_name = cameras[0];
  }


  size(camera_width, camera_height);

  cam = new Capture(this, camera_name);

  println(camera_name + " Initialized");

  cam.start();

}


void captureEvent(Capture c) {
  c.read();
}


ArrayList<Contour> all_contours, four_v_contours;
PImage _dst;

void findBoard() {

  opencv = new OpenCV(this, camera_width, camera_height);

  opencv.loadImage(cam);

  println("opencv: "+opencv);

  opencv.gray();
  opencv.threshold(70);
  // _dst = opencv.getOutput();

  all_contours = opencv.findContours();

  for (Contour contour : all_contours) {

    if (contour.getPolygonApproximation().getPoints().size() == 4) {
      four_v_contours.add(contour);
    }
  }

}




void mouseClicked(MouseEvent event) {

  super.mouseClicked(event);

  if(x1==0){
    x1=event.getX();
    y1=event.getY();
  }else if(x2==0){
    x2=event.getX();
    y2=event.getY();
  }else if(x4==0){
    x4=event.getX();
    y4=event.getY();
  }else if(x3==0){
    x3=event.getX();
    y3=event.getY();
  }
}

void draw() {

  if (cam.available() == true) {
    cam.read();
  }

  findBoard();


  if (four_v_contours.size() > 0){

    for (Contour contour : four_v_contours) {

      println("contour " );

      stroke(0, 255, 0);
      contour.draw();

      stroke(255, 0, 0);
      beginShape();
      for (PVector point : contour.getPolygonApproximation().getPoints()) {
        vertex(point.x, point.y);
      }
      endShape();
    }
  }

  image(cam,0,0);

  // opencv.loadImage(cam);
  // opencv.updateBackground();

  noFill();
  stroke(255, 0, 0);
  strokeWeight(1);

  line(x1,y1,x2,y2);
  line(x1,y1,x3,y3);
  line(x2,y2,x4,y4);
  line(x3,y3,x4,y4);


  strokeWeight(25);


  int board_size = 19;

  float step_x = (x2-x1)/board_size;
  float step_y = - (y1-y3)/board_size;

  float shift_x = (x3-x1)/board_size - (x4-x2)/board_size;
  float shift_y = (y2-y1)/board_size - (y4-y3)/board_size;

  for (int i=0;i<board_size;i++){
    for (int j=0;j<board_size;j++){


      float _ax = x1 + (x3-x1)*i/board_size;
      float _bx = x2 + (x4-x2)*i/board_size;

      float _ay = y1 + (y3-y1)*i/board_size;
      float _by = y2 + (y4-y2)*i/board_size;

      float _cx = x1 + (x2-x1)*j/board_size;
      float _dx = x3 + (x4-x3)*j/board_size;

      float _cy = y1 + (y2-y1)*j/board_size;
      float _dy = y3 + (y4-y3)*j/board_size;

      int res = intersect(_ax, _ay, _bx, _by, _cx, _cy, _dx, _dy);

      if (res != 2) {
        println("dont intersect: " + i + "  " + j);
      }



      // line((int)(x1+(x3-x1)*(i)/19),(int)( y1+(y3-y1)*(i)/19),(int)(x2+(x4-x2)*(i)/19), (int)(y2+(y4-y2)*(i)/19));
      //   line((int)(x1+(x2-x1)*(i)/19),(int)( y1+(y2-y1)*(i)/19),(int)(x3+(x4-x3)*(i)/19), (int)(y3+(y4-y3)*(i)/19));

      // color c = ;

      stroke(cam.get( (int)x, (int)y));

      point(x, y);

    }
  }

  // opencv.dilate();
  // opencv.erode();


  // ArrayList<Contour> contours = opencv.findContours();

  // for (Contour contour : contours) {
  //   contour = contour.getPolygonApproximation();
  //   if (contour.area() > 5000.0) {

  //     System.out.println(contour.area());
  //     contour.draw();
  //   }
  // }

}




int intersect(float x1, float y1, float x2, float y2, float x3, float y3, float x4, float y4){

  float a1, a2, b1, b2, c1, c2;
  float r1, r2 , r3, r4;
  float denom, offset, num;

  // Compute a1, b1, c1, where line joining points 1 and 2
  // is "a1 x + b1 y + c1 = 0".
  a1 = y2 - y1;
  b1 = x1 - x2;
  c1 = (x2 * y1) - (x1 * y2);

  // Compute r3 and r4.
  r3 = ((a1 * x3) + (b1 * y3) + c1);
  r4 = ((a1 * x4) + (b1 * y4) + c1);

  // Check signs of r3 and r4. If both point 3 and point 4 lie on
  // same side of line 1, the line segments do not intersect.
  if ((r3 != 0) && (r4 != 0) && same_sign(r3, r4)){
    return DONT_INTERSECT;
  }

  // Compute a2, b2, c2
  a2 = y4 - y3;
  b2 = x3 - x4;
  c2 = (x4 * y3) - (x3 * y4);

  // Compute r1 and r2
  r1 = (a2 * x1) + (b2 * y1) + c2;
  r2 = (a2 * x2) + (b2 * y2) + c2;

  // Check signs of r1 and r2. If both point 1 and point 2 lie
  // on same side of second line segment, the line segments do
  // not intersect.
  if ((r1 != 0) && (r2 != 0) && (same_sign(r1, r2))){
    return DONT_INTERSECT;
  }

  //Line segments intersect: compute intersection point.
  denom = (a1 * b2) - (a2 * b1);

  if (denom == 0) {
    return COLLINEAR;
  }

  if (denom < 0){
    offset = -denom / 2;
  }
  else {
    offset = denom / 2 ;
  }

  // The denom/2 is to get rounding instead of truncating. It
  // is added or subtracted to the numerator, depending upon the
  // sign of the numerator.
  num = (b1 * c2) - (b2 * c1);
  if (num < 0){
    x = (num - offset) / denom;
  }
  else {
    x = (num + offset) / denom;
  }

  num = (a2 * c1) - (a1 * c2);
  if (num < 0){
    y = ( num - offset) / denom;
  }
  else {
    y = (num + offset) / denom;
  }

  // lines_intersect
  return DO_INTERSECT;
}


boolean same_sign(float a, float b){

  return (( a * b) >= 0);
}