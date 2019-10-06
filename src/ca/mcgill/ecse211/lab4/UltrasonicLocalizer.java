package ca.mcgill.ecse211.lab4;

import static ca.mcgill.ecse211.lab4.Resources.*;


public class UltrasonicLocalizer extends UltrasonicController{

    
    
    @Override
    public void processUSData(int distance) {
      filter(distance);
      
      
      
    }

    @Override
    public int readUSDistance() {
      return this.distance;
    }
  }



