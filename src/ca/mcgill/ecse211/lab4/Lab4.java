package ca.mcgill.ecse211.lab4;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import ca.mcgill.ecse211.lab4.OdometerExceptions;

public class Lab4 { 
	
	/*Initializing any variables this class may need */
	public static final double WHEEL_RAD = 1.60;
	public static final double TRACK = 18.55;
	public static final double[] positionWaypoints = {1,0,2,1,2,2,0,2,1,1};
	public static boolean falling_edge = false;
	
	/*Initializing Motors, and LCD */
	public static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	public static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final TextLCD lcd = LocalEV3.get().getTextLCD();
	
	/**Main Class, runs upon robot start*/
	public static void main(String[] args) throws ca.mcgill.ecse211.lab4.OdometerExceptions {
		int buttonChoice;
		/* Setting up the Odometer, display, and US Sensor */
		Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
		
		Display odometryDisplay = new Display(lcd);
		
		do {
			/* Hear we clear the display, and set up the menu options we want */
			lcd.clear();
			
			lcd.drawString("< Left | Right>", 0, 0);
			lcd.drawString("       |       ", 0, 1);
			lcd.drawString("Rising | Falling", 0, 2);
			lcd.drawString("Edge   | edge", 0, 3);
			
			buttonChoice = Button.waitForAnyPress();
		} while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);
		
		/* Begin the Odometer and Odometer Correction threads */
		Thread odoThread = new Thread(odometer);
		odoThread.start();
		Thread odoDisplayThread = new Thread(odometryDisplay);
		odoDisplayThread.start();
		
		falling_edge = false; 
		/* If the user decides to run obstacle avoidance, this boolean will be set to true for use later on */
		if (buttonChoice == Button.ID_RIGHT) {
			falling_edge = true;
			//System.out.println("Right button chosen, running falling edge");
		}
		
		/* This new threads starts the navigation thread. The obstacle avoidance boolean variable and Ultrasonic sensor instance is also passed */
		
		new Thread() {
			public void run() {
				try {
					if(falling_edge == true) {
						//System.out.println("Running falling edge");
						UltrasonicLocalizer.falling_Edge(leftMotor, rightMotor, WHEEL_RAD, WHEEL_RAD, TRACK);
					}
					else if (falling_edge == false) {
						//System.out.println("Running rising edge");
						UltrasonicLocalizer.rising_Edge(leftMotor, rightMotor, WHEEL_RAD, WHEEL_RAD, TRACK);
					}
				} catch (OdometerExceptions e) {
					e.printStackTrace();
				}
			}
		}.start();

		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		
		new Thread() {
			public void run() {
				try {
					LightLocalizer.light_Localizer(leftMotor, rightMotor, WHEEL_RAD, WHEEL_RAD, TRACK);
				} catch (OdometerExceptions e) {
					e.printStackTrace();
				}
			}
		}.start();
		 
		
		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		
		System.exit(0);
	}

}
