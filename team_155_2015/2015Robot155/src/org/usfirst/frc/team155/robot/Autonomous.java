package org.usfirst.frc.team155.robot;

public class Autonomous {
	
	DRIVE155 robotDrive;
	Lift155 robotLift;
	robotMap155 robotMap;
	
	//arcade state
	final int DRIVE0= 0;
	final int TURN180= 1;
	final int DRIVE180= 2;
	
	//mecanum state
	final int DRIVEBACK= 0;
	final int DRIVESIDE= 1;
	final int DRIVEFOWARD= 2;
	final int STOP= 3;
	public int state= 0;
	
	public Autonomous(DRIVE155 drive, Lift155 lift, robotMap155 robot) {
		robotDrive = drive;
		robotLift = lift;
		robotMap = robot;
	}
	
	public void run() {
		double speed;
		double heading = 0;
		if (robotLift.measureDistance()> 36) {
			speed= .5;
		}
		else speed= 0;
		//robotLift.measureDistance();
		robotDrive.driveStraight(heading,speed);
	}
	public void autoMode(){
		double speed= 0;
		double heading= 0;
		
		switch(state){
			case DRIVE0:
				speed= .5;
				heading= 0;
				if (robotLift.measureDistance()<= 36) {
					state= TURN180;
				}
				break;
			case TURN180: 
				speed= 0;
				heading= 180;
				if (Math.abs(robotDrive.roboGyro.getAngle()-180)<=5){
					state= DRIVE180;
				}
				break;
			case DRIVE180:
				speed= .5;
				heading= 180;
				if (robotLift.measureDistance()<= 36) {
					state= STOP;
				}
				break;
			case STOP: 
				speed= 0;
				heading= 180;
				break;
		}
		robotDrive.driveStraight(heading,speed);
				
				
		}
	public void autoForward(){
		double speed= 0;
		double heading= 0;
		double direction= 0;
		
		switch(state){
			case DRIVEBACK:
				speed= .5;
				heading= 0;
				direction= 270;
				if (robotLift.measureDistance()<= 36) {
					state= TURN180;
				}
				break;
			case DRIVESIDE: 
				speed= .5;
				heading= 0;
				direction= 180;
				if (Math.abs(robotDrive.roboGyro.getAngle()-180)<=5){
					state= DRIVE180;
				}
				break;
			case DRIVEFOWARD:
				speed= .5;
				heading= 0;
				direction= 90;
				if (robotLift.measureDistance()<= 36) {
					state= STOP;
				}
				break;
			case STOP: 
				speed= 0;
				heading= 0;
				break;
		}
		robotDrive.driveMecanum(heading,speed,direction);
		}
	}


