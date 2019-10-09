package ca.mcgill.ecse211.lab4;
import ca.mcgill.ecse211.lab4.UltrasonicController;
import ca.mcgill.ecse211.lab4.Odometer;
import ca.mcgill.ecse211.lab4.UltrasonicPoller;
import static ca.mcgill.ecse211.lab4.Resources.*;
import lejos.hardware.*;
import lejos.hardware.sensor.EV3UltrasonicSensor;


public class UltrasonicLocalizer implements Runnable {
  

  private static final int WALL_THRESHOLD = 40;
  private static final int FAR_PING_THRESHOLD = 3;
  private int correctionAngle=168;
  
  private float[] usData = new float[US_SENSOR.sampleSize()];
  private static LocalizationType locType;


  public UltrasonicLocalizer(LocalizationType localType) {
    locType = localType;
    // TODO Auto-generated constructor stub
  }




  public static double getAngleDiff(double a, double b) {
      // Convert to vectors and use dot product to compute angle in between
      return Math.abs(Math.acos(Math.cos(a) * Math.cos(b) + Math.sin(a) * Math.sin(b)));
  }

  
  private int fetchUSData() {
    US_SENSOR.fetchSample(usData, 0);
    int distanceFound = (int) (usData[0]*100);
    return distanceFound;
  }

  public static enum LocalizationType {
      FALLING_EDGE, RISING_EDGE
  }
  
  /**
   * This method allows the conversion of a distance to the total rotation of each wheel need to cover that distance.
   * 
   * @param radius
   * @param distance
   * @return
   */
  private static int convertDistance(double radius, double distance) {
    return (int) ((180.0 * distance) / (Math.PI * radius));
  }

  private static int convertAngle(double radius, double width, double angle) {
    return convertDistance(radius, Math.PI * width * angle / 360.0);
  }
  /**
   * Falling Edge method
   */
  private void fallingEdge() {
    double angle1;
    double angle2;
    double deltaTheta=0;
    
    leftMotor.setSpeed(100);
    rightMotor.setSpeed(100);


    //if distance is smaller than d + noise (toEdge), turn towards the wall
    while(fetchUSData() < distanceEdge + toEdge) {
      leftMotor.backward();
      rightMotor.forward();
    
    }
    
    //turn towards wall again if distance is greater than d, detected falling edge
    while(fetchUSData() > distanceEdge) {
      leftMotor.backward();
      rightMotor.forward();
    }
    
    //record first angle, represents alpha
    angle1 = odometer.getXYT()[2];
    Sound.beep();

    //now turn other way and look for second falling edge
    while(fetchUSData() < distanceEdge + toEdge) {
      leftMotor.forward();
      rightMotor.backward();
      
    
    }
    
    //turn away, look for second falling edge
    while(fetchUSData() > distanceEdge) {
      leftMotor.forward();
      rightMotor.backward();
     
    }
    
    //record second angle measured, represents beta
    angle2 = odometer.getXYT()[2];
    Sound.beep();

    
    //compare angles from each wall, find deltaTheta based on which is bigger
    if(angle1 < angle2) {
      deltaTheta = 45-((angle1+angle2)/2);
      deltaTheta+=correctionAngle; //without correctionAngle it goes to 0 degrees facing the wrong way, correctionAngle turns it back forwards
      
    }
    
    else if(angle1 > angle2) {
      deltaTheta = 215-((angle1+angle2)/2);
    }
    
    //add change in angle needed to current angle
    deltaTheta+=odometer.getXYT()[2];
    
    //rotate accordingly
    leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, deltaTheta), true);
    rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, deltaTheta), false);

  }
  
  /**
   * Rising Edge method
   */
  private void risingEdge() {
    double angle1;
    double angle2;
    double deltaTheta=0;
    double turnAngle;
    
    leftMotor.setSpeed(100);
    rightMotor.setSpeed(100);

    //check if us sensor detects wall farther away than d, if yes, turn towards wall
    while(fetchUSData() > distanceEdge) {
      leftMotor.backward();
      rightMotor.forward();
    }
    
    //if closer to wall than d+noise, turn towards wall
    while(fetchUSData() < distanceEdge + toEdge) {
      leftMotor.backward();
      rightMotor.forward();
    
    }
    
   
    //record first angle, alpha
    angle1 = odometer.getXYT()[2];
    
    //perform turns in other direction to get the second angle
    while(fetchUSData() > distanceEdge) {
      leftMotor.forward();
      rightMotor.backward();
     
    }

    while(fetchUSData() < distanceEdge + toEdge) {
      leftMotor.forward();
      rightMotor.backward();
      
    }
    
  
    //record the second angle, beta
    angle2 = odometer.getXYT()[2];
    
    //compare angles from each wall, find deltaTheta based on which is bigger
    if(angle1 < angle2) {
      deltaTheta = 45-((angle1+angle2)/2);
      deltaTheta+=correctionAngle;//without correctionAngle it goes to 0 degrees facing the wrong way, correctionAngle turns it back forwards
      
      
    }
    
    else if(angle1 > angle2) {
      deltaTheta = 220-((angle1+angle2)/2);

    }
    //add change in angle needed to current angle
    turnAngle = deltaTheta + odometer.getXYT()[2];

    //rotate accordingly
    leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, turnAngle), true);
    rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, turnAngle), false);

  }


  @Override
  public void run() {
    //get localization type from constructor and run method accordingly
    if (locType == LocalizationType.FALLING_EDGE) {
      fallingEdge();
    }
    else {
      
      risingEdge();
    }    
  
  }
}



