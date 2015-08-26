//package io.alatalab.glassbead;

import gab.opencv.*;

import java.util.*;

import processing.core.*;
import processing.video.Capture;
import themidibus.ControlChange;
import themidibus.MidiBus;
import processing.video.*;

public class Prelude  {
  //Movie video;//
   Random rand = new Random();
  Capture cam1,cam2;
  int camera_width = Integer.valueOf(Props.getValue("camera_width"));
  int camera_height = Integer.valueOf(Props.getValue("camera_height"));
  int log_camera_width = Integer.valueOf(Props.getValue("log_camera_width"));
  int log_camera_height = Integer.valueOf(Props.getValue("log_camera_height"));
  static int previous_number_of_objects = 0, number_of_objects = 0,
      previous_number_of_contours = 0, number_of_contours = 0;
  Integer[] rows_to_play = new Integer[] { 1, 2,
      3 , 4, 5, 8, 9 };
  

  static Queue<Integer> rows_played = new LinkedList<Integer>();

  @Override
  public void stop() {
    super.stop();
  }

  public void movieEvent(Movie m) {//
    m.read();//
  }

  public void setup() {
    size(camera_width, camera_height/2);

    cam1 = new Capture(this, camera_width/2, camera_height/2,Props.getValue("camera_name"), 30);
    cam2 = new Capture(this, camera_width/2, camera_height/2,
        Props.getValue("log_camera_name"), 30);
  //  video = new Movie(this, "/Users/chro/Desktop/1-2-3e.mov");//
    opencv = new OpenCV(this,camera_width, camera_height);

    opencv.startBackgroundSubtraction(5, 3, 0.5);
    cam1.start();
    //cam2.start();
    //video.loop();
    // video.play();

  }

  void personEntered(int delta_number_of_person, int delta_number_of_contours) {
    // playColumn(number_of_person);
    System.out.println(delta_number_of_person + ";"
        + delta_number_of_contours);
    if (delta_number_of_contours > 10000)
      makeGlitch(delta_number_of_contours / 500);
    
    if (delta_number_of_person > 0) {
      for (int i = 0; i < delta_number_of_person; i++) {
        
        int row = rows_to_play[rand.nextInt(rows_to_play.length)];    
        playRow(row);
        rows_played.add(row);
      }
    } else if (delta_number_of_person > 0) {
      for (int i = 0; i < delta_number_of_person; i++) {
        muteRow(rows_played.poll());
      }
    }

  }

  void makeGlitch(int value) {
    sendMidiMessage(complicationBus, 1, 1, value);

  }
   void resetSound(){
     makeGlitch(0);
     for (int i = 0; i < rows_to_play.length; i++) {
    muteRow(rows_to_play[i]);
  }
   }
  public void draw() {
  
    
    PImage image = new PImage(log_camera_width+camera_width, camera_height);
    image.set(0, 0, cam1);
    image.set(log_camera_width, 0, cam2);

  //  opencv.loadImage(image);
    
    image(image,0,0);
    opencv.loadImage(image);
    
    
    opencv.updateBackground();

    opencv.dilate();
    opencv.erode();

    noFill();
    stroke(255, 0, 0);
    strokeWeight(1);

    ArrayList<Contour> contours = opencv.findContours();
    number_of_contours=contours.size();
    number_of_objects = 0;

    for (Contour contour : contours) {
      contour = contour.getPolygonApproximation();
      if (contour.area() > 2000.0) {
        number_of_objects++;
        System.out.println(contour.area());
        contour.draw();
      }
    }
    
        if(number_of_contours<10 && previous_number_of_contours<10) {
          resetSound();
        //  delay(1000);
          return;
        }
    personEntered(number_of_objects - previous_number_of_objects,
        number_of_contours - previous_number_of_contours);
    previous_number_of_objects = number_of_objects;
    previous_number_of_contours = number_of_contours;
    // opencv = new OpenCV(this, cam);
    // delay(10*1000);
    // play3Line();
    // personEntered(1);

    // image(opencv.getSnapshot(), 0, 0);

    /*
     * for(int i=10;i<127;i++){ personEntered(i); delay(20*1000);
     * System.out.println(i); }
     */

  }

  @Override
  public void startAll() {
    sendMidiMessage(newMoveBus, 2, 72, 127);

  }

  @Override
  public void stopAll() {
    makeGlitch(0);
    sendMidiMessage(newMoveBus, 2, 73, 127);
    System.out.println("stopped");
  }

  @Override
  public void muteRow(int i) {
    sendMidiMessage(newMoveBus, 2, 73 + i, 127);

  }

  @Override
  public void playRow(int i) {
    switch (i) {
    case 1:
      sendMidiMessage(newMoveBus, 2, 1 + rand.nextInt(24), 127);
    case 2:
      sendMidiMessage(newMoveBus, 2, 25 + rand.nextInt( 13), 127);
    case 3:
      sendMidiMessage(newMoveBus, 2, 39 + rand.nextInt( 7), 127);
    case 4:
      sendMidiMessage(newMoveBus, 2, 47 + rand.nextInt(9), 127);
    case 5:
      sendMidiMessage(newMoveBus, 2, 57 + rand.nextInt(4), 127);
    case 8:
      sendMidiMessage(newMoveBus, 2, 62 +rand.nextInt( 4), 127);
    case 9:
      sendMidiMessage(newMoveBus, 2, 67 + rand.nextInt( 4), 127);
    }

  }

  @Override
  public void playColumn(int i) {
    return;

  }
  
  public void captureEvent(Capture c) {
      c.read();
    }

}
