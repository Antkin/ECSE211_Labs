package ca.mcgill.ecse211.lab4;

import java.util.Arrays;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;

public class LightLocalizer {
	
	/* Initializing sensor and variables */
	private static Odometer odometer;
	private static EV3ColorSensor light_Sensor;
	private static float[] current_Light_Value = new float[1];
	private static float[] light_samples = new float[50];
	private static int increment;
	private static float median;
	private static double[] final_position = {0,0};
	
	/** This is our median finder method, for increasing light sensor reliability */
	public static float median_Finder(float[] light_samples) {
		  float median;
		  Arrays.sort(light_samples);
		  median = light_samples[(light_samples.length/2)];
		  return median;
	  }
	
	public static void light_Localizer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, double leftRadius, double rightRadius, double track) throws OdometerExceptions {
		Port light_Sensor_Port = LocalEV3.get().getPort("S1");
		light_Sensor = new EV3ColorSensor(light_Sensor_Port);
		light_Sensor.setFloodlight(6);
		odometer = Odometer.getOdometer();
		
		for(increment = 0; increment < 50; increment++) {
			light_Sensor.getRedMode().fetchSample(current_Light_Value, 0);
			light_samples[increment] = current_Light_Value[0];
		}
		
		leftMotor.setAcceleration(75);
		rightMotor.setAcceleration(75);
		
		leftMotor.forward();
		rightMotor.forward();
		leftMotor.setSpeed(75);
		rightMotor.setSpeed(75);
		
		while(true) {
			light_Sensor.getRedMode().fetchSample(current_Light_Value, 0);
			if(increment >= 49) {
				increment = 0;
			}
			light_samples[increment] = current_Light_Value[0];
			increment++;
			median = median_Finder(light_samples);
			
			if(Math.abs(current_Light_Value[0] - median) >= .06) {
				odometer.setY(0);
				Sound.beep();
				leftMotor.setSpeed(0);
				rightMotor.setSpeed(0);
				break;
			}
			//System.out.println("Testing1");
		}
		
		while(leftMotor.getRotationSpeed() != 0 && rightMotor.getRotationSpeed() != 0);
		
		Driver.drive(leftMotor, rightMotor, leftRadius, rightRadius, track, -15);
		
		Driver.turn(leftMotor, rightMotor, leftRadius, rightRadius, track, 90);
		
		leftMotor.setAcceleration(75);
		rightMotor.setAcceleration(75);
		
		leftMotor.forward();
		rightMotor.forward();
		leftMotor.setSpeed(75);
		rightMotor.setSpeed(75);
		
		while(true) {
			light_Sensor.getRedMode().fetchSample(current_Light_Value, 0);
			if(increment >= 49) {
				increment = 0;
			}
			light_samples[increment] = current_Light_Value[0];
			increment++;
			median = median_Finder(light_samples);
			
			if(Math.abs(current_Light_Value[0] - median) >= .06) {
				odometer.setY(0);
				Sound.beep();
				leftMotor.setSpeed(0);
				rightMotor.setSpeed(0);
				break;
			}
			//System.out.println("Testing2");
		}
		
		while(leftMotor.getRotationSpeed() != 0 && rightMotor.getRotationSpeed() != 0);
		
		Driver.drive(leftMotor, rightMotor, leftRadius, rightRadius, track, -15);
		
		odometer.setX(-15);
		odometer.setY(-15);
		
		Navigation.navigationControl(leftMotor, rightMotor, leftRadius, rightRadius, track, final_position);
		System.out.println("Finished nav, turning now.");
		Navigation.turnTo(leftMotor, rightMotor, leftRadius, rightRadius, track, 0, odometer.getTheta());
		
	}
}
