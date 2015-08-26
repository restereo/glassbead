import gab.opencv.*;
OpenCV opencv ;

PImage _image;

void setup (){
  size(1280,960);
  opencv = new OpenCV(this,1280,960 );
  _image = loadImage("/Users/restereo/Desktop/0.jpg");
  opencv.loadImage(_image);
    
}


  public void draw() {
  
    image(_image,0,0);

  }
