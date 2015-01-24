package org.usfirst.frc.team155.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Encoder;

public class Lift155 {
	robotMap155 robotSystem;
	Joystick rightStick;
	DoubleSolenoid sol;
	DigitalInput lowLimit;
	DigitalInput highLimit;
	Victor liftDrive;
	AnalogInput rangeFinder;
	PIDController liftMotorPID;
	final double kP = 0;
	final double kI = 0;
	final double kD = 0;
	Encoder liftEncoder;
	final int LOWEST_LEVEL = 0;
	final int ONE_CRATE = 100;
	final int TWO_CRATE = 200;
	final int THREE_CRATE = 300;
	final int FOUR_HEIGHT = 400;
	
	public final int GROUND_LEVEL = 0;
	public final int CARRY_BOX = 1;
	public final int STACK_BOX = 2;
		
	
	public Lift155(robotMap155 robot, Joystick right){
		robotSystem = robot;
		rightStick = right;
		sol = new DoubleSolenoid(robot.GRIPPER_SIDE_A, robot.GRIPPER_SIDE_B);
        lowLimit = new DigitalInput(robot.LOW_LIMIT);
        highLimit = new DigitalInput(robot.HIGH_LIMIT);
		liftDrive = new Victor(robot.LIFT_MOTOR);
		rangeFinder = new AnalogInput(robot.RANGE_FINDER);
		liftEncoder = new Encoder(robotSystem.LIFT_ENCODER_A, robotSystem.LIFT_ENCODER_B);
		liftEncoder.setPIDSourceParameter(PIDSource.PIDSourceParameter.kDistance);
		liftMotorPID = new PIDController(kP, kI, kD, liftEncoder, liftDrive);
		
	}
	
	//TeamCheer Involves Waffles
	
	//LIFT METHOD
	
	private void manualLift(){
		
		if  (highLimit.get() || lowLimit.get())
    		if (lowLimit.get() && (rightStick.getY()<0))
    				liftDrive.set(rightStick.getY());
    		else if  (highLimit.get() && (rightStick.getY()>0))
    				liftDrive.set(rightStick.getY());
    		else 
    			liftDrive.set(0);
    	else 
    		liftDrive.set(rightStick.getY());
    	
	}
	
	public void autoLift(double setPoint){
		
		if  (highLimit.get() || lowLimit.get())
    		if (lowLimit.get() && (liftMotorPID.get()<0)){
    			liftMotorPID.enable();
    			liftMotorPID .setSetpoint(setPoint);
    			}
    		else if  (highLimit.get() && (liftMotorPID.get()>0)){
    			liftMotorPID.enable();
    			liftMotorPID .setSetpoint(setPoint);
    		}
    		else {
    			liftMotorPID.disable();
    			liftDrive.set(0);
    		}
    	else {
    		liftMotorPID.enable();
    		liftMotorPID .setSetpoint(setPoint);// You Will Never Find A Pony More Majestic Than Waffles
    		}
	}
		public boolean onTarget(){
			return (liftMotorPID.onTarget());
		}
	
	
	//GRABBER method
	private void grabber(){
		
		if (rightStick.getTrigger()) 
    		sol.set(DoubleSolenoid.Value.kForward);
    	else sol.set(DoubleSolenoid.Value.kReverse);
		
	}
	
	public double measureDistance(){
		
		double distance;
		double scale =106.2;//48 inches/.452 volts
		distance = rangeFinder.getVoltage()*scale;
		return distance;
	}
	public void run(){
		//call lift....
		manualLift();
		
		//call grabber....
		grabber();
		
		SmartDashboard.putNumber("Distance", measureDistance());
	}

}
