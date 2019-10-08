package ca.mcgill.ecse211.lab4;
import ca.mcgill.ecse211.lab4.UltrasonicController;
import ca.mcgill.ecse211.lab4.Odometer;
import ca.mcgill.ecse211.lab4.UltrasonicPoller;
import static ca.mcgill.ecse211.lab4.Resources.*;
import lejos.hardware.*;
import lejos.hardware.sensor.EV3UltrasonicSensor;


public class UltrasonicLocalizer {
  

  private static final int WALL_THRESHOLD = 40;
  private static final int FAR_PING_THRESHOLD = 3;
  private final Odometer odo;
  private final Navigation navig;
  private final EV3UltrasonicSensor us;
  private final LocalizationType locType;
  private int farPingCount = 0;
  private int lastValidDistance = 255;
  private int distance;
  private float[] usData;


  public UltrasonicLocalizer(Odometer odo, Navigation navig, EV3UltrasonicSensor us, LocalizationType locType) {
      
      this.odo = odo;
      this.navig = navig;
      this.us = us;
      this.locType = locType;
   }

  public void doLocalization() {
         
      if (locType == LocalizationType.FALLING_EDGE) {
          // rotate the robot until it sees no wall
          rotateToNoWall(-1);
          // keep rotating until the robot sees a wall, then latch the angle
          rotateToWall(-1);
          double angleA = odo.getTheta();
          // switch direction and wait until it sees no wall
          rotateToNoWall(1);
          // keep rotating until the robot sees a wall, then latch the angle
          rotateToWall(1);
          double angleB = odo.getTheta();
          // angleA is clockwise from angleB, so assume the average of the
          // angles to the right of angleB is 45 degrees past 'north'
          double angle = getAngleDiff(angleB, angleA) / 2 + angleA;
          // update the odometer orientation
          odo.setTheta(odo.getTheta() - Math.PI / 4 - angle);
      }
      
      else {
          // rotate the robot until it sees a wall
          rotateToWall(-1);
          // keep rotating until the robot sees no wall, then latch the angle
          rotateToNoWall(-1);
          double angleA = odo.getTheta();
          // switch direction and wait until it sees a wall
          rotateToWall(1);
          // keep rotating until the robot sees no wall, then latch the angle
          rotateToNoWall(1);
          double angleB = odo.getTheta();
          // angleA is clockwise from angleB, so assume the average of the
          // angles to the right of angleB is 225 degrees past 'north'
          double angle = getAngleDiff(angleB, angleA) / 2 + angleA;
          // update the odometer orientation
          odo.setTheta(odo.getTheta() + Math.PI / 4 - angle);
      }
      // Turn to zero to verify results
      navig.turnTo(0);
      navig.waitUntilDone();
  }

  private void rotateToWall(double direction) {
      //us.fetchSample(usData, 1); // acquire data
    //  distance = (int) (usData[0] * 100.0); // extract from buffer, cast to int
      boolean foundWall = false;
      direction = Math.signum(direction) * 0.9 * Math.PI;
      do {
          // turn the maximum amount
          navig.turnBy(direction);
          // wait until a wall isn't found or navigation is done
          while (!(foundWall = getFilteredData(distance) <= WALL_THRESHOLD) && navig.isNavigating());
      } while (!foundWall);
      // make sure that navigation is stopped
      navig.abort();
  }

  private void rotateToNoWall(double direction) {
   // us.fetchSample(usData, 1); // acquire data 
    boolean foundWall = false;
    direction = Math.signum(direction) * 0.9 * Math.PI;
    //distance = (int) (usData[1] * 100.0); // extract from buffer, cast to int
    do {
          // turn the maximum amount
        navig.turnBy(direction);
          // wait until a wall isn't found or navigation is done
        while ((foundWall = getFilteredData(distance) <= WALL_THRESHOLD) && navig.isNavigating());
      }
    while (foundWall);
      // make sure that navigation is stopped
      navig.abort();
  }

  // Get the absolute difference between two angles
  public static double getAngleDiff(double a, double b) {
      // Convert to vectors and use dot product to compute angle in between
      return Math.abs(Math.acos(Math.cos(a) * Math.cos(b) + Math.sin(a) * Math.sin(b)));
  }
  
  
  public int readUSDistance() {
    return this.distance;
  }

  private int getFilteredData(int distance) {
      // do a ping
     // Sound.beep();
      // wait for the ping to complete
      try {
          Thread.sleep(50);
      } catch (InterruptedException e) {
      }
      // there will be a delay here
      //int distance = us.getDistance();
      // Check for nothing found
      if (distance == 255) {
        this.distance = distance;

          // If nothing found for a while, return that
          if (farPingCount >= FAR_PING_THRESHOLD) {
              return 255;
          } else {
              // Else add to the count and return last valid
              farPingCount++;
              return lastValidDistance;
          }
      } 
      
      else if (farPingCount > 0) {
          // If something found decrement nothing found count (min zero)
          farPingCount--;
      }
      // Set last valid distance to this one
      lastValidDistance = distance;
      // Return the distance
      return distance;
  }

  public static enum LocalizationType {
      FALLING_EDGE, RISING_EDGE
  }
  }



