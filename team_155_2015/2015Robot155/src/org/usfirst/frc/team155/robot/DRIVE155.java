package org.usfirst.frc.team155.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Gyro;

public class DRIVE155 {
		robotMap155 robotSystem;
		
		//JOYSTICKS
	    Joystick leftStick = new Joystick(1);
	    Joystick rightStick = new Joystick(2);

	    //MOTORS
	    public Talon left_front;
	    public Talon right_front;
	    public Talon left_back;
	    public Talon right_back;
	    public RobotDrive myrobot;
	    
	    //DRIVE ENCODERS
	    public Encoder Front_Right_Encoder;
	    public Encoder Front_Left_Encoder;
	    public Encoder Back_Right_Encoder;
	    public Encoder Back_Left_Encoder;
	    
	    //YAW RATE SENSOR
	    public Gyro roboGyro;
	    
	    
	    //CONTROLLERS

	    
	    public DRIVE155(Joystick left, Joystick right, robotMap155 robot) {
	        robotSystem = robot;
	        
	        //JOYSTICKS - NEEDS MORE APPROPRIATE NAMES
	        leftStick = left;
	        rightStick = right;

	        //MOTORS
	        left_front = new Talon(robotSystem.DRIVE_LEFT_FRONT);
	        right_front = new Talon(robotSystem.DRIVE_RIGHT_FRONT);
	        left_back = new Talon(robotSystem.DRIVE_LEFT_BACK);
	        right_back = new Talon(robotSystem.DRIVE_RIGHT_BACK);
	        

	        myrobot = new RobotDrive(left_front,left_back,right_front,right_back);
	        
	        myrobot.setInvertedMotor(RobotDrive.MotorType.kFrontLeft, true);
	        myrobot.setInvertedMotor(RobotDrive.MotorType.kFrontRight, true);
	        myrobot.setInvertedMotor(RobotDrive.MotorType.kRearLeft, true);
	        myrobot.setInvertedMotor(RobotDrive.MotorType.kRearRight, true);
	        //ENCODERS
	        Front_Right_Encoder = new Encoder(robotSystem.FRONT_RIGHT_ENCODER_A, robotSystem.FRONT_RIGHT_ENCODER_B);
	        Front_Right_Encoder.setDistancePerPulse(.1);
	        //YAW RATE (GYRO)
	        roboGyro = new Gyro(robotSystem.GYRO);
	        
	        //CONTROLLER
	        
	    }
	    
	    //drive modes
	    private void arcade(){
	    	
	    	myrobot.arcadeDrive(leftStick);
	    	
	    		
	    	
	    }

	    private void mecanum_fieldOriented(){
	    	myrobot.mecanumDrive_Cartesian(leftStick.getX(), leftStick.getY(), rightStick.getX(), roboGyro.getAngle());
	    }
	    //drive selector

	    public void run(){
	    	arcade();
	    	System.out.println("gyro = " + roboGyro.getAngle());
	    	
	    	System.out.println("distance = " + Front_Right_Encoder.get() + " Rate = " + Front_Right_Encoder.getRate());
		}
	    

}    


