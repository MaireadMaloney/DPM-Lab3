package ca.mcgill.ecse211.lab4;


import lejos.hardware.Button;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
import static ca.mcgill.ecse211.lab4.Resources.*;



public class Main {

  // ultrasonic sensor instance from resources.
  //public static final SampleProvider usDistance = US_SENSOR.getMode("Distance");
  //public static final float[] usData = new float[usDistance.sampleSize()];

  // Minimum distance to send to poller.
  private static EV3ColorSensor colorSensor = Resources.colorSensor;  //private SampleProvider intensityVal = colorSensor.getRedMode();
  public static final double MIN_DISTANCE = 14.0;
  public static final TextLCD lcd = LocalEV3.get().getTextLCD();
  //private static Odometer odo;
  //private static Navigation navig;
  public static UltrasonicLocalizer usl;
  public static LightLocalizer2 lsl;
  private static Navigation navigator = Resources.navigator;


  public static void main(String[] args) {
    int buttonChoice;
    Odometer odo = new Odometer();
    
    Display odometryDisplay = new Display(lcd); // No need to change.

  
     
    do {
      // Clear the display.
      lcd.clear();

      // Ask the user whether the motors should drive in a square or float.
      lcd.drawString("< Left | Right >", 0, 0);
      lcd.drawString("       |        ", 0, 1);
      lcd.drawString("Falling| Rising ", 0, 2);
      lcd.drawString("Edge   | Edge  ", 0, 3);
      lcd.drawString("       |     ", 0, 4);

      buttonChoice = Button.waitForAnyPress(); // Record the user's choice (left or right press).
    }

    while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);

    if (buttonChoice == Button.ID_LEFT) { // Falling edge has been selected.
      lcd.clear();
   // Declaring ultrasonic and controller variables.
 
      // Start odometer thread.
      Thread odoThread = new Thread(odo);
      odoThread.start();
      Thread odoDisplayThread = new Thread(odometryDisplay);
      odoDisplayThread.start();
      
      //new Thread(new UltrasonicPoller()).start();
      Thread navigatorThread = new Thread(navigator);
      navigatorThread.start();
      
      usl = new UltrasonicLocalizer(odo, navigator, US_SENSOR, UltrasonicLocalizer.LocalizationType.FALLING_EDGE); 
      Thread uslThread = new Thread(usl);
      uslThread.start();
      
      Button.waitForAnyPress();
      
      LightLocalizer2 lsl = new LightLocalizer2(odo, navigator, colorSensor);
      Thread lslThread = new Thread(lsl);
      lslThread.start();
      
      Button.waitForAnyPress();
      System.exit(0);

  

    } else { // Rising edge has been selected 
      LCD.clear();

      // Declaring ultrasonic and controller variables.
      //new Thread(new UltrasonicPoller()).start();

      //localizer instance, rising edge
      //usl.doLocalization();
      Thread odoDisplayThread = new Thread(odometryDisplay);
      odoDisplayThread.start();
   // Start ultrasonic thread.
      

      // Start odometer thread.
      Thread odoThread = new Thread(odo);
      odoThread.start();
    //new Thread(new UltrasonicPoller()).start();
      navigator.start();
      
      usl = new UltrasonicLocalizer(odo, navigator, US_SENSOR, UltrasonicLocalizer.LocalizationType.RISING_EDGE);
      
      Thread uslThread = new Thread(usl);
      uslThread.start();

      Button.waitForAnyPress();
      // perform the light sensor localization
      lsl = new LightLocalizer2(odo, navigator, colorSensor);
      //lsl.doLocalization();
      // Start display thread.
      //Thread odoDisplayThread = new Thread(odometryDisplay);
      //odoDisplayThread.start();
      //
      

      // Set up navigation points.
 


      // Start navigation thread.

    }

    while (Button.waitForAnyPress() != Button.ID_ESCAPE);
    System.exit(0);

  }


}
