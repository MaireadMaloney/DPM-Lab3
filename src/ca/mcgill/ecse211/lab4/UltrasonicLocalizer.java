package ca.mcgill.ecse211.lab4;

import ca.mcgill.ecse211.lab4.Odometer;
import static ca.mcgill.ecse211.lab4.Resources.*;


public class UltrasonicLocalizer extends UltrasonicController{

    public static final int NOISE_MARGIN = 5;
    public static final int IDEAL_READ_THRESHOLD = 31;
    
    @Override
    public void processUSData(int distance) {
      filter(distance);
      int minDistance = 255;
      boolean isSeeingWall = true;
      if (distance <= minDistance) {
        minDistance = distance;
      }
      else {
        isSeeingWall = false;
      }
      
      if (isSeeingWall == false) {
        leftMotor.stop();
        rightMotor.stop();
        //To-Do set x y to distance and set theta to 180 degrees 
      }
    }
    
    public void fallingEdge() {
      
    }
    
    public void risingEdge() {
      
    }
    
    @Override
    public int readUSDistance() {
      return this.distance;
    }
  }



