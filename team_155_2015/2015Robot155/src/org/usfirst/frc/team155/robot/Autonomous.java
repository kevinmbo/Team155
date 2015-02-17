package org.usfirst.frc.team155.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Autonomous {

	DRIVE155 robotDrive;
	Lift155 robotLift;
	robotMap155 robotMap;
	CameraThread robotVision;
	// Vision155 robotVision;

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
	// start
	final int START2 = 0;
	final int WAIT2 = 1;
	final int FIRSTTOTE2 = 2;
	final int LIFTTOTE2 = 3;
	final int DRIVEFORWARD2 = 4;
	final int TURN90 = 5;
	final int DRIVESTRAIGHT2 = 6;
	final int STOP2 = 7;

	// public Autonomous(DRIVE155 drive, Lift155 lift, robotMap155 robot,
	// Vision155 vision) {
	public Autonomous(DRIVE155 drive, Lift155 lift, robotMap155 robot,
			CameraThread vision) {
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

	public void centerTote() {
		if (robotVision.hasFoundTote())
			robotDrive.centerYellowTote(300, 0, robotVision.getTotePosition());

		else
			robotDrive.mecanumstop();
		SmartDashboard.putBoolean("foundtote", robotVision.hasFoundTote());
	}

	public void driveToAutoZone() {
		double distance = 84;
		if (robotDrive.DriveStraightDistance(distance)) {
			// robotDrive.PIDDisable();
			// robotDrive.mecanumstop();
		} else
			robotDrive.DriveStraightDistance(distance);

	}

	public void driveToTote() {
		double midPoint = -40;
		double finalPoint = -80;
		double fullSpeed = -.25;
		double slowSpeed = -.125;
		double speed;

		if (robotDrive.EncoderDistance() > midPoint)
			speed = fullSpeed;
		else if (robotDrive.EncoderDistance() > finalPoint)
			speed = slowSpeed;
		else
			speed = 0;

		if (robotVision.hasFoundTote())
			robotDrive.centerYellowTote(300, speed,
					robotVision.getTotePosition());

		else
			robotDrive.mecanumstop();
		SmartDashboard.putBoolean("foundtote", robotVision.hasFoundTote());
	}

	public boolean driveToToteRangeFinder() {
		System.out.println("In driveToToteRangeFinder");

		// double midPoint = 40;
		double finalPoint = 8;
		double fullSpeed = -.18;
		// double slowSpeed = -.25;
		double speed;
		double toteDistance;
		boolean reachTote = false;

		toteDistance = robotLift.measureDistance();
		System.out.println("toteDistance = " + toteDistance);
		SmartDashboard.putNumber("RangeFinderDistance-driveToToteRangeFinder",
				toteDistance);

		if (toteDistance > finalPoint) {
			System.out.println("toteDistance is > 8");
			speed = fullSpeed;
			System.out.println("speed = " + speed);

			if (robotVision.hasFoundTote()) {
				System.out.println("Tote found!! goto centerYellowTote");
				System.out.println("goalposition = 300");
				System.out.println("speed = " + speed);
				robotDrive.centerYellowTote(300, speed, robotVision.getTotePosition());
			} else {
				System.out.println("Tote NOT found!! goto mecanumstop");
				robotDrive.mecanumstop();
			}
		} else {
			System.out.println("toteDistance is <= 8, Reached the tote!!!");
			speed = 0;
			System.out.println("speed = " + speed);
			reachTote = true;
		}
		SmartDashboard.putBoolean("foundtote", robotVision.hasFoundTote());
		return reachTote;

	}

	public void autoMode() {
		double speed = 0;
		double heading = 0;
		double direction = 90;

		switch (state) {
		case DRIVEFORWARD1:

			break;
		case LIFTTOTE1:
			speed = 0;
			heading = 0;
			direction = 90;
			double carryHeight;
			// lift the tote

			if (readyToCarry == false) {
				carryHeight = robotLift.GROUND_LEVEL;
				if (robotLift.onTarget() == true)
					readyToCarry = true;
			} else
				carryHeight = robotLift.CARRY_BOX;
			if ((readyToCarry == true) && (robotLift.onTarget())
					&& (carryHeight == robotLift.CARRY_BOX))
				state = DRIVEFORWARD1;
			robotLift.autoLift(carryHeight);

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

	public void autoLine() {
		double speed = 0;
		double heading = 0;
		double direction = 90;

		switch (state) {
		case START2:
			System.out.println("in START2");
			System.out.println("speed = " + speed);
			System.out.println("heading = " + heading);
			System.out.println("direction = " + direction);

			startTimeDRIVE = Timer.getFPGATimestamp();
			state = WAIT2;

		case WAIT2:
			System.out.println("in WAIT2");
			speed = 0;
			heading = 0;
			direction = 90;
			System.out.println("speed = " + speed);
			System.out.println("heading = " + heading);
			System.out.println("direction = " + direction);

			robotDrive.driveMecanum(heading, speed, direction);
			if ((Timer.getFPGATimestamp()) - (startTimeDRIVE) > 1) {
				state = FIRSTTOTE2;
				startTimeDRIVE = Timer.getFPGATimestamp();
			}

			break;
		case FIRSTTOTE2:
			System.out.println("in FIRSTTOTE2");
			heading = 0;
			direction = 0;
			System.out.println("speed = " + speed);
			System.out.println("heading = " + heading);
			System.out.println("direction = " + direction);
			SmartDashboard.putNumber("RangeFinderDistance-FIRSTTOTE2",
					robotLift.measureDistance());
			System.out.println("RangeFinderDistance = "
					+ robotLift.measureDistance());

			if (robotLift.measureDistance() <= 8) {
				speed = 0;
				state = LIFTTOTE2;
			} else
				speed = .25;

			System.out.println("speed = " + speed);
			System.out.println("heading = " + heading);
			System.out.println("direction = " + direction);

			robotDrive.driveMecanum(heading, speed, direction);

			startTimeDRIVE = Timer.getFPGATimestamp();
			// SmartDashboard.putNumber("RangeFinderDistance",
			// robotLift.measureDistance());
			break;

		case LIFTTOTE2:
			System.out.println("in LIFTTOTE2");

			speed = 0;
			heading = 0;
			direction = 90;
			System.out.println("speed = " + speed);
			System.out.println("heading = " + heading);
			System.out.println("direction = " + direction);

			// lift the tote
			/*
			 * double carryHeight;
			 * 
			 * if (readyToCarry == false) { carryHeight=robotLift.GROUND_LEVEL;
			 * if (robotLift.onTarget() == true) readyToCarry = true; } else
			 * carryHeight=robotLift.CARRY_BOX; if
			 * ((readyToCarry==true)&&(robotLift
			 * .onTarget())&&(carryHeight==robotLift.CARRY_BOX)) { if
			 * (BOX_COUNTER<3) state = DRIVEFORWARD2; else state = TURN90; }
			 * 
			 * robotLift.autoLift(carryHeight);
			 */

			// Pauses code to simulate lift
			if ((Timer.getFPGATimestamp()) - (startTimeDRIVE) > 4) {
				BOX_COUNTER++;
				if (BOX_COUNTER < 3) {
					state = DRIVEFORWARD2;
				} else
					state = TURN90;
			}
			break;

		case DRIVEFORWARD2:
			System.out.println("in DRIVEFORWARD2");
			driveToToteRangeFinder();
			if (driveToToteRangeFinder())
				state = LIFTTOTE2;
			startTimeDRIVE = Timer.getFPGATimestamp();
			break;

		case TURN90:
			System.out.println("in TURN90");
			speed = 0;
			heading = -90;
			direction = 0;
			System.out.println("heading = " + heading);
			System.out.println("speed = " + speed);
			System.out.println("direction = " + direction);

			robotDrive.driveMecanum(heading, speed, direction);

			System.out.println("Math.abs(robotDrive.getGyro() = "
					+ Math.abs(robotDrive.getGyro()));
			if (Math.abs(robotDrive.getGyro() - heading) < 5) {
				System.out
						.println("Math.abs(robotDrive.getGyro() - heading is < 5");
				System.out.println("PIDEnable and Reset Encoder");

				robotDrive.PIDEnable();
				robotDrive.EncoderReset();
				state = DRIVESTRAIGHT2;
			}
			break;

		case DRIVESTRAIGHT2:
			System.out.println("in DRIVESTRAIGHT2");
			System.out.println("heading = " + heading);
			System.out.println("speed = " + speed);
			System.out.println("direction = " + direction);

			robotDrive.DriveStraightDistance(84);

			if (robotDrive.DriveStraightDistance(84))
				state = STOP2;
			break;

		case STOP2:
			System.out.println("in STOP2");
			speed = 0;
			heading = 0;
			direction = 180;
			System.out.println("heading = " + heading);
			System.out.println("speed = " + speed);
			System.out.println("direction = " + direction);

			robotDrive.PIDDisable();
			break;
		}

	}

	public void displayRangeFinder() {
		SmartDashboard.putNumber("RangeFinderDistance",
				robotLift.measureDistance());

	}
}
