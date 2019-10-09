package ca.mcgill.ecse211.lab4;

import static ca.mcgill.ecse211.lab4.Resources.*;

import static ca.mcgill.ecse211.lab4.Resources.WHEEL_RAD;
import static ca.mcgill.ecse211.lab4.Resources.leftMotor;
import static ca.mcgill.ecse211.lab4.Resources.rightMotor;
import lejos.hardware.Sound;
import lejos.hardware.sensor.EV3ColorSensor;

public class LightLocalizer2 implements Runnable{
  private float sampleData[] = new float [1];
  private float brightness;
  private Odometer odometer = Resources.odometer;
  private Navigation navigator = Resources.navigator;
  private EV3ColorSensor colorSensor = Resources.colorSensor;

 
  
//
//  public LightLocalizer2(Odometer odo, Navigation navigator, EV3ColorSensor colorSensor) {
//   
//   }
  
  private static int convertDistance(double radius, double distance) {
    return (int) ((180.0 * distance) / (Math.PI * radius));
  }

  private static int convertAngle(double radius, double width, double angle) {
    return convertDistance(radius, Math.PI * width * angle / 360.0);
  }
  
  @Override
  public void run() {
    Sound.beep();
    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);
    int angle = 45;
    leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, angle), true);
    rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, angle), false);
    
    boolean lineReached = false;
    
    while(!lineReached) {
      leftMotor.forward();
      rightMotor.forward();
      colorSensor.getRedMode().fetchSample(sampleData, 0);
      brightness = sampleData[0];
      
      if(brightness<26.00) {
        lineReached = true;
        leftMotor.stop();
        rightMotor.stop();
        
      }
    }
    

    
  }

  
  

}
