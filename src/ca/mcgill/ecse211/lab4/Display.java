package ca.mcgill.ecse211.lab4;

import java.text.DecimalFormat;
import lejos.hardware.lcd.TextLCD;

public class Display extends Thread {

  private Odometer odo;
  private UltrasonicPoller usPoller;
  private TextLCD lcd;
  private double[] position;
  private final long DISPLAY_PERIOD = 25;
  private long timeout = Long.MAX_VALUE;

  /**
   * This is the class constructor
   * 
   * @param odoData
   * @throws OdometerExceptions
   */
  public Display(TextLCD lcd) {
    odo = Odometer.getOdometer();
    this.lcd = lcd;
  }

  /**
   * This is the overloaded class constructor
   * 
   * @param odoData
   * @throws OdometerExceptions
   */
  public Display(TextLCD lcd, long timeout) {
    odo = Odometer.getOdometer();
    this.timeout = timeout;
    this.lcd = lcd;
  }

  public void run() {
    usPoller = UltrasonicPoller.getInstance();
    lcd.clear();

    long updateStart, updateEnd;

    long tStart = System.currentTimeMillis();
    do {
      updateStart = System.currentTimeMillis();

      // Retrieve x, y and theta information.
      position = odo.getXYT();



      // Print x,y, and theta information
      DecimalFormat numberFormat = new DecimalFormat("######0.00");
      lcd.drawString("X: " + numberFormat.format(position[0]), 0, 0);
      lcd.drawString("Y: " + numberFormat.format(position[1]), 0, 1);
      lcd.drawString("T: " + numberFormat.format(position[2]), 0, 2);


      if (usPoller != null) { // Shows US sensor reading for object distance.
        lcd.drawString("Distance: " + numberFormat.format(usPoller.distance), 0, 3);
      }


      // This ensures that the data is updated only once every period.
      updateEnd = System.currentTimeMillis();
      if (updateEnd - updateStart < DISPLAY_PERIOD) {
        try {
          Thread.sleep(DISPLAY_PERIOD - (updateEnd - updateStart));
        } catch (InterruptedException e) {
          e.printStackTrace();
        }
      }
    } while ((updateEnd - tStart) <= timeout);

  }
}