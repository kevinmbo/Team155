package org.usfirst.frc.team155.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Autonomous {

	DRIVE155 robotDrive;
	Lift155 robotLift;
	robotMap155 robotMap;
	CameraThread robotVision;
	//Vision155 robotVision;

	// arcade state
	final int DRIVE0 = 0;
	final int TURN180 = 1;
	final int DRIVE180 = 2;
	final int LIFT_TOTE = 3;

	// Mech mode 1
	final int DRIVEFORWARD1 = 0;
	final int LIFTTOTE1 = 1;
	final int DRIVEBACK1 = 2;
	final int DRIVESIDE1 = 3;
	final int STOP1 = 4;
	int BOX_COUNTER = 0;
	// mecanum state
	final int DRIVEBACK = 0;
	final int DRIVESIDE = 1;
	final int DRIVEFOWARD = 2;
	final int STOP = 3;
	public int state = 0;
	public boolean TOTE_SWITCH = false;
	public double startTimeDRIVE;
	boolean readyToCarry = false;
	DigitalInput toteSwitch;

	//public Autonomous(DRIVE155 drive, Lift155 lift, robotMap155 robot, Vision155 vision) {
	public Autonomous(DRIVE155 drive, Lift155 lift, robotMap155 robot, CameraThread vision) {
		robotDrive = drive;
		robotLift = lift;
		robotMap = robot;
		robotVision = vision;
	}

	public void run() {
		double speed;
		double heading = 0;
		if (robotLift.measureDistance() > 36) {
			speed = .5;
		} else
			speed = 0;
		// robotLift.measureDistance();
		robotDrive.driveStraight(heading, speed);
	}
	
	public void centerTote(){
	if (robotVision.hasFoundTote())	
		robotDrive.centerYellowTote(300, 0, robotVision.getTotePosition());
		
	else robotDrive.mecanumstop();	
	SmartDashboard.putBoolean("foundtote", robotVision.hasFoundTote());
	}

	public void autoMode() {
		double speed = 0;
		double heading = 0;
		double direction = 90;

		switch (state) {
		case DRIVEFORWARD1:
			speed = .5;
			heading = 0;
			direction = 90;
			if (BOX_COUNTER == 0)
				robotLift.autoLift(robotLift.GROUND_LEVEL);
			else
				robotLift.autoLift(robotLift.STACK_BOX);
			if (toteSwitch.get()) {
				state = LIFTTOTE1;
			}
			break;
		case LIFTTOTE1:
			speed = 0;
			heading = 0;
			direction = 90;
			// lift the tote
			if (BOX_COUNTER == 0) {
				robotLift.autoLift(robotLift.CARRY_BOX);
				if (robotLift.onTarget() == true) {
					state = DRIVEBACK1;
					BOX_COUNTER++;
				}
			}

			else {
				if (readyToCarry == false) {
					robotLift.autoLift(robotLift.GROUND_LEVEL);
					if (robotLift.onTarget() == true)
						readyToCarry = true;
					robotLift.autoLift(robotLift.CARRY_BOX);
				} else {
					if (robotLift.onTarget() == true) {
						if (BOX_COUNTER == 3)
							state = STOP;
						else {
							state = DRIVEBACK1;
							BOX_COUNTER++;
						}
					}
				}
			}

			if (BOX_COUNTER == 3)
				state = STOP;
			else {
				state = DRIVEBACK1;
				BOX_COUNTER++;
			}
			break;
		case DRIVEBACK1:
			speed = .5;
			heading = 0;
			direction = 270;
			if ((Timer.getFPGATimestamp()) - (startTimeDRIVE) > .5) {
				state = DRIVESIDE1;
				startTimeDRIVE = Timer.getFPGATimestamp();
			}
			break;
		case DRIVESIDE1:
			speed = .5;
			heading = 0;
			direction = 180;
			if ((Timer.getFPGATimestamp()) - (startTimeDRIVE) > 1)
				if (robotLift.measureDistance() <= 36) {
					state = DRIVEFORWARD1;
				}

			break;
		case STOP1:
			speed = 0;
			heading = 0;
			direction = 180;
			break;
		}
		robotDrive.driveMecanum(heading, speed, direction);

	}

	public void autoForward() {
		double speed = 0;
		double heading = 0;
		double direction = 0;

		switch (state) {
		case DRIVEFOWARD:
			speed = .5;
			heading = 0;
			direction = 90;
			if (robotLift.measureDistance() <= 36) {
				state = TURN180;
			}
			break;
		case DRIVEBACK:
			speed = .5;
			heading = 0;
			direction = 180;
			if (Math.abs(robotDrive.roboGyro.getAngle() - 180) <= 5) {
				state = DRIVE180;
			}
			break;
		case DRIVESIDE:
			speed = .5;
			heading = 0;
			direction = 90;
			if (robotLift.measureDistance() <= 36) {
				state = STOP;
			}
			break;
		case STOP:
			speed = 0;
			heading = 0;
			break;
		}
		robotDrive.driveMecanum(heading, speed, direction);
	}
	
}
