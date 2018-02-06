/*
P * OdometryCorrection.java
 */
package ca.mcgill.ecse211.odometer;

import java.util.Arrays;

import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;

public class OdometryCorrection implements Runnable {
  private static final long CORRECTION_PERIOD = 10;
  private Odometer odometer;
  private float[] light_samples;
  private float[] current_light_sample;
  private int increment = 0;
  private EV3ColorSensor sensor;
  long line_detected_time;
  public int line_count;
  private double[] position = new double[3];
  private double local_y;
  private double local_x;
  private double mod_calc;
  private float median;

  /**
   * This is the default class constructor. An existing instance of the odometer is used. This is to
   * ensure thread safety.
   * 
   * @throws OdometerExceptions
   */
  public OdometryCorrection() throws OdometerExceptions {
	//Here we set up the sensor, set up a float array to hold the value of the double, and make a line_count variable
    this.odometer = Odometer.getOdometer();
    Port port = LocalEV3.get().getPort("S1");
    sensor = new EV3ColorSensor(port);
    light_samples = new float[50];
    current_light_sample = new float[1];
    for(increment = 0; increment < 50; increment++) {
    	sensor.getRedMode().fetchSample(current_light_sample, 0);
    	light_samples[increment] = current_light_sample[0];
    }
    increment = 0; 
  }
  
  public float median_Finder(float[] light_samples) {
	  float median;
	  Arrays.sort(light_samples);
	  median = light_samples[(light_samples.length/2)];
	  return median;
  }

  /**
   * Here is where the odometer correction code should be run.
   * 
   * @throws OdometerExceptions
   */
  // run method (required for Thread)
  public void run() {
    long correctionStart, correctionEnd;
    //Setting the backlight
    sensor.setFloodlight(6);
    try {
		Thread.sleep(5000);
	} 
    catch (Exception e) {
	}
    while (true) {
      correctionStart = System.currentTimeMillis();
      //In order to make a "filter" for the values, we import the current x and y value to know how close we are to the next line
      position = odometer.getXYT();
      //The sensor fetches the current value and assigns it to sample
      sensor.getRedMode().fetchSample(current_light_sample, 0);
      if(increment == 49) {
    	  increment = 0;
      }
      light_samples[increment] = current_light_sample[0];
      increment++;
      median = median_Finder(light_samples);
      System.out.println(median+" "+current_light_sample[0]);
      //Here we have our detection setup, if the light sensor falls below .26 it is likely to be a black line.
      //Also included some extra code to try and prevent instances where the sensor would beep several times
      if(Math.abs(current_light_sample[0] - median) >= .05) {
    	  if((position[2] >= 315 && position[2] <= 359)||(position[2] >= 0 && position[2] <= 45)||(position[2] >= 135 && position[2] <= 225)) {
    		  local_y = position[1];
    		  local_y = (local_y)/30.48;
    		  mod_calc = Math.floor(local_y);
    		  local_y = local_y - mod_calc;
    		  if(mod_calc == 0) {
    			  odometer.setY(0);
    			  Sound.beep();
    		  }
    		  else if(local_y >= .75 && local_y <= .99) {
    			  local_y = (mod_calc+1)*(30.48);
    			  odometer.setY(local_y);
    			  Sound.beep();
    		  }
    		  else if(local_y >= 0.00 && local_y <= .25) {
    			  local_y = (mod_calc)*(30.48);
    			  odometer.setY(local_y);
    			  Sound.beep();
    		  }
    	  }
    	  if((position[2] >= 45 && position[2] <= 135)||(position[2] >= 225 && position[2] <= 315)) {
    		  local_x = position[0];
    		  local_x = (local_x)/30.48;
    		  mod_calc = Math.floor(local_x);
    		  local_x = local_x - mod_calc;
    		  if(mod_calc == 0) {
    			  odometer.setX(0);
    			  Sound.beep();
    		  }
    		  else if(local_x >= .75 && local_x <= .99) {
    			  local_x = (mod_calc+1)*(30.48);
    			  odometer.setX(local_x);
    			  Sound.beep();
    		  }
    		  else if(local_x >= 0.00 && local_x <= .25) {
    			  local_x = (mod_calc)*(30.48);
    			  odometer.setX(local_x);
    			  Sound.beep();
    		  }
    	  }
      }
      //Part of the filter, used to help us prevent the sensor from detecting the same line twice
 
      // this ensure the odometry correction occurs only once every period
      correctionEnd = System.currentTimeMillis();
      if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
        try {
          Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));
        } catch (InterruptedException e) {
          // there is nothing to be done here
        }
      }
    }
  }
}
