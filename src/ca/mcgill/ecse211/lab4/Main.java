package ca.mcgill.ecse211.lab4;


import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.robotics.SampleProvider;
import static ca.mcgill.ecse211.lab4.Resources.*;

public class Main {

  // ultrasonic sensor instance from resources.
  public static final SampleProvider usDistance = US_SENSOR.getMode("Distance");
  public static final float[] usData = new float[usDistance.sampleSize()];

  // Minimum distance to send to poller.
  public static final double MIN_DISTANCE = 14.0;
  public static final TextLCD lcd = LocalEV3.get().getTextLCD();

  public static void main(String[] args) {
    int buttonChoice;


    Display odometryDisplay = new Display(lcd); // No need to change.
    Navigation navigator = new Navigation(odometer, leftMotor, rightMotor, WHEEL_RAD, TRACK, TILE_SIZE);

    do {
      // Clear the display.
      lcd.clear();

      // Ask the user whether the motors should drive in a square or float.
      lcd.drawString("< Left | Right >", 0, 0);
      lcd.drawString("       |        ", 0, 1);
      lcd.drawString("Simple | with  ", 0, 2);
      lcd.drawString("navig. | avoid-   ", 0, 3);
      lcd.drawString("       | ance ", 0, 4);

      buttonChoice = Button.waitForAnyPress(); // Record the user's choice (left or right press).
    }

    while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);

    if (buttonChoice == Button.ID_LEFT) { // Simple navigation has been selected.

      // Start odometer thread.
      Thread odoThread = new Thread(odometer);
      odoThread.start();

      // Start display thread.
      Thread odoDisplayThread = new Thread(odometryDisplay);
      odoDisplayThread.start();

   

      // Start navigation thread.
      Thread navigatorThread = new Thread(navigator);
      navigatorThread.start();


    } else { // Navigation w/ avoidance has been selected, so we include an ultrasonic thread as well.
      LCD.clear();

      // Declaring ultrasonic and controller variables.
      UltrasonicPoller usPoller = UltrasonicPoller.getInstance();

      // Start odometer thread.
      Thread odoThread = new Thread(odometer);
      odoThread.start();

      // Start display thread.
      Thread odoDisplayThread = new Thread(odometryDisplay);
      odoDisplayThread.start();
      //
      // Start ultrasonic thread.
      Thread ultrasonicThread = new Thread(usPoller);
      ultrasonicThread.start();

      // Set up navigation points.
 


      // Start navigation thread.
      Thread navigatorThread = new Thread(navigator);
      navigatorThread.start();

    }

    while (Button.waitForAnyPress() != Button.ID_ESCAPE);
    System.exit(0);

  }


}