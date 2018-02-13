package ca.mcgill.ecse211.lab4;

import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class UltrasonicLocalizer {
	
	/* Initializing the sensor and any variables */
	private static int rotate_Speed = 150;
	private static double[] position = new double[3];
	private static OdometerData odometer;
	private static final Port us_Port = LocalEV3.get().getPort("S2");
	private static double alpha, beta;
	private static int detected_Distance;
	private static double fix_Theta;
	

	static SensorModes us_Sensor = new EV3UltrasonicSensor(us_Port);
	final static SampleProvider us_Distance = us_Sensor.getMode("Distance");
	public static float[] us_sample = new float[us_Distance.sampleSize()];
	
	public static void falling_Edge(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, double leftRadius, double rightRadius, double track) throws OdometerExceptions {
		odometer = Odometer.getOdometer();
		us_Distance.fetchSample(us_sample, 0);
		detected_Distance = (int) (us_sample[0] * 100);
		
		leftMotor.setAcceleration(125);
		rightMotor.setAcceleration(125);
		
		leftMotor.forward();
		rightMotor.backward();
		leftMotor.setSpeed(rotate_Speed);
		rightMotor.setSpeed(rotate_Speed);
		
		while(detected_Distance > 40) {
			us_Distance.fetchSample(us_sample, 0);
			detected_Distance = (int) (us_sample[0] * 100);
		}
		Sound.beep();
		
		position = odometer.getXYT();
		alpha = position[2];
		
		leftMotor.stop(true);
		rightMotor.stop(false);
		
		Driver.turn(leftMotor, rightMotor, leftRadius, rightRadius, track, -60);
		
		rightMotor.forward();
		leftMotor.backward();
		rightMotor.setSpeed(rotate_Speed);
		leftMotor.setSpeed(rotate_Speed);
		
		us_Distance.fetchSample(us_sample, 0);
		detected_Distance = (int) (us_sample[0] * 100);
		
		while(detected_Distance > 40) {
			us_Distance.fetchSample(us_sample, 0);
			detected_Distance = (int) (us_sample[0] * 100);
		}
		Sound.beep();
		
		position = odometer.getXYT();
		beta = position[2];
		
		leftMotor.stop(true);
		rightMotor.stop(false);
		
		if(alpha < beta) {
			fix_Theta = 225 - (alpha+beta)/2;
		}
		else if(alpha >= beta) {
			fix_Theta = 45 - (alpha+beta)/2;
		}
		
		//System.out.println("Alpha is "+alpha+"     Beta is "+beta);
		//System.out.println("Fix theta is "+fix_Theta);
		//System.out.println("This is the falling edge method");
		
		odometer.setTheta(odometer.getTheta() + fix_Theta);
		
		Navigation.turnTo(leftMotor, rightMotor, leftRadius, rightRadius, track, 0.0, odometer.getTheta());
		
		
	}
	
	public static void rising_Edge(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, double leftRadius, double rightRadius, double track) throws OdometerExceptions {
		odometer = Odometer.getOdometer();
		us_Distance.fetchSample(us_sample, 0);
		detected_Distance = (int) (us_sample[0] * 100);
		
		leftMotor.setAcceleration(125);
		rightMotor.setAcceleration(125);
		
		leftMotor.forward();
		rightMotor.backward();
		leftMotor.setSpeed(rotate_Speed);
		rightMotor.setSpeed(rotate_Speed);
		
		while(detected_Distance < 40) {
			us_Distance.fetchSample(us_sample, 0);
			detected_Distance = (int) (us_sample[0] * 100);
		}
		Sound.beep();
		
		position = odometer.getXYT();
		alpha = position[2];
		
		leftMotor.stop(true);
		rightMotor.stop(false);
		
		Driver.turn(leftMotor, rightMotor, leftRadius, rightRadius, track, -60);
		
		rightMotor.forward();
		leftMotor.backward();
		rightMotor.setSpeed(rotate_Speed);
		leftMotor.setSpeed(rotate_Speed);
		
		
		
		us_Distance.fetchSample(us_sample, 0);
		detected_Distance = (int) (us_sample[0] * 100);
		
		while(detected_Distance < 40) {
			us_Distance.fetchSample(us_sample, 0);
			detected_Distance = (int) (us_sample[0] * 100);
		}
		Sound.beep();
		
		position = odometer.getXYT();
		beta = position[2];
		
		leftMotor.stop(true);
		rightMotor.stop(false);
		
		if(alpha < beta) {
			fix_Theta = 45 - (alpha+beta)/2;
		}
		else if(alpha >= beta) {
			fix_Theta = 225 - (alpha+beta)/2;
		}
		
		//System.out.println("Alpha is "+alpha+"     Beta is "+beta);
		//System.out.println("Fix theta is "+fix_Theta);
		//System.out.println("This is the rising edge method");
		
		odometer.setTheta(odometer.getTheta() + fix_Theta);
		
		Navigation.turnTo(leftMotor, rightMotor, leftRadius, rightRadius, track, 0.0, odometer.getTheta());
	}
	
}
