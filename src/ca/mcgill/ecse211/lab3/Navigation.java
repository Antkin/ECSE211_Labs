package ca.mcgill.ecse211.lab3;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

public class Navigation {
	private static OdometerData odometer;
	private static double deltaX;
	private static double deltaY;
	private static double turnToTheta;
	private static double currentTheta;
	private static double deltaTheta;
	private static double distance;
	private static int increment;
	private static double[] robotPosition = new double[3];
	private static double[] nextWayPoint = new double[2];
	
	
	public static void navigationControl(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
		      double leftRadius, double rightRadius, double track, double[] positionWaypoints, SampleProvider us_Distance, boolean obstacle_Avoidance) throws OdometerExceptions {
		odometer = Odometer.getOdometer();
		for(increment = 0; increment < 5; increment++) {
			System.out.println("This is the "+increment+" turn");
			robotPosition = odometer.getXYT();
			System.out.println("Current X position is: "+robotPosition[0]+"     Current Y position is: "+robotPosition[1]+"       Current Theta is: "+robotPosition[2]);
			nextWayPoint[0] = positionWaypoints[increment*2] * 30.48;
			nextWayPoint[1] = positionWaypoints[increment*2 + 1] * 30.48;
			deltaX = nextWayPoint[0] - robotPosition[0];
			deltaY = nextWayPoint[1] - robotPosition[1];
			System.out.println("Delta X is: "+deltaX);
			System.out.println("Delta Y is: "+deltaY);
			turnToTheta = Math.atan(deltaX/deltaY);
			turnToTheta = Math.toDegrees(turnToTheta);
			if(deltaX > 0 && deltaY < 0) {
				turnToTheta = turnToTheta + 180;
			}
			if(deltaX < 0 && deltaY < 0) {
				turnToTheta = turnToTheta - 180;
			}
			if(turnToTheta < 0) {
				turnToTheta = turnToTheta + 360;
			}
			if(deltaX == 0) {
				if(deltaY > 0) {
					turnToTheta = 0;
				}
				else if(deltaY < 0) {
					turnToTheta = 180;
				}
			}
			distance = Math.sqrt(Math.pow(deltaX, 2)+Math.pow(deltaY, 2));
			currentTheta = robotPosition[2];
			turnTo(leftMotor, rightMotor, leftRadius, rightRadius, track, turnToTheta, currentTheta);
			travelTo(leftMotor, rightMotor, leftRadius, rightRadius, track, distance, us_Distance, obstacle_Avoidance);
			odometer.setXYT(nextWayPoint[0], nextWayPoint[1], turnToTheta);
		}
		
	}
	
	public static void travelTo(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
		      double leftRadius, double rightRadius, double track, double distance, SampleProvider us_Distance, boolean obstacle_Avoidance) {
		if(obstacle_Avoidance) {
			Driver.obstacle_Driver(leftMotor, rightMotor, leftRadius, rightRadius, track, distance, us_Distance);
		}
		else {
		Driver.drive(leftMotor, rightMotor, leftRadius, rightRadius, track, distance);
		}
	}
	
	public static void turnTo(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
		      double leftRadius, double rightRadius, double track, double turnToTheta, double currentTheta) {
		if(currentTheta >= 355 || currentTheta <= 5) {
			currentTheta = 0;
		}
		deltaTheta  = turnToTheta - currentTheta;
		if(deltaTheta > 180) {
			deltaTheta = deltaTheta - 360;
		}
		else if(deltaTheta < -180) {
			deltaTheta = deltaTheta + 360;
		}
		System.out.println("CurrentTheta is: "+currentTheta);
		System.out.println("turnToTheta is: "+turnToTheta);
		System.out.println("DeltaTheta is"+deltaTheta);
		if (deltaTheta < 0) {
			System.out.println("Turning left!");
			Driver.turn(leftMotor, rightMotor, leftRadius, rightRadius, track, deltaTheta);
		}
		else if (deltaTheta > 0) {
			System.out.println("Turning right!");
			Driver.turn(leftMotor, rightMotor, leftRadius, rightRadius, track, deltaTheta);
		}
		
	}
}
