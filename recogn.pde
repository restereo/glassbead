
int x1, x2, x3, x4, y1, y2, y3, y4;

void setup() {

  rate = 30;

  camera_width = 1280;
  camera_height = 760;

  size(camera_width, camera_height);

  cam = new Capture(this, camera_width, camera_height, "USB", rate);

  cam.start();

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

  image(cam,0,0);

  // opencv.loadImage(cam);
  // opencv.updateBackground();

  line(x1,y1,x2,y2);
  line(x1,y1,x3,y3);
  line(x2,y2,x4,y4);
  line(x3,y3,x4,y4);

   // for(int i=0;i<19;i++){
   //   line((int)(x1+(x3-x1)*(i)/19),(int)( y1+(y3-y1)*(i)/19),(int)(x2+(x4-x2)*(i)/19), (int)(y2+(y4-y2)*(i)/19));
   //   line((int)(x1+(x2-x1)*(i)/19),(int)( y1+(y2-y1)*(i)/19),(int)(x3+(x4-x3)*(i)/19), (int)(y3+(y4-y3)*(i)/19));

   //    //cam.get(x, y);

   // }


  // opencv.dilate();
  // opencv.erode();

  noFill();
  stroke(255, 0, 0);
  strokeWeight(1);


  // ArrayList<Contour> contours = opencv.findContours();

  // for (Contour contour : contours) {
  //   contour = contour.getPolygonApproximation();
  //   if (contour.area() > 5000.0) {

  //     System.out.println(contour.area());
  //     contour.draw();
  //   }
  // }

}
