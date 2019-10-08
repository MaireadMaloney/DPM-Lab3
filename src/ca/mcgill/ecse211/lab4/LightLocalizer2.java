package ca.mcgill.ecse211.lab4;

import static ca.mcgill.ecse211.lab4.Resources.TRACK;
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

 
  

  public LightLocalizer2(Odometer odo, Navigation navigator, EV3ColorSensor ls) {
    ls.getRedMode().fetchSample(sampleData, 0);
    brightness = sampleData[0]*100;
       // set the sensor flood light to red
       ls.setCurrentMode("Red");
   }
  
  private static int convertDistance(double radius, double distance) {
    return (int) ((180.0 * distance) / (Math.PI * radius));
  }

  private static int convertAngle(double radius, double width, double angle) {
    return convertDistance(radius, Math.PI * width * angle / 360.0);
  }
  
  @Override
  public void run() {
    Sound.beep();
    int angle = (int)(45.00*Math.PI)/180;
    leftMotor.forward();
    
  }

  
  

}
