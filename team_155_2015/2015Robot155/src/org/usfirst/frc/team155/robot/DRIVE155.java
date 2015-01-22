package org.usfirst.frc.team155.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DRIVE155 {
		robotMap155 robotSystem;
		SmartDashboard sDash;
		
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
	    double Kp = .05;
        double Ki = 0;
        double Kd = 0;
        double PIDGyroOut;
	    public PIDController PIDGyro;
	    
	    public DRIVE155(Joystick left, Joystick right, robotMap155 robot) {
	        robotSystem = robot;
	        sDash = new SmartDashboard();
	        
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
	        Front_Left_Encoder = new Encoder(robotSystem.FRONT_LEFT_ENCODER_A, robotSystem.FRONT_LEFT_ENCODER_B);
	        Front_Left_Encoder.setDistancePerPulse(.1); 
	        Back_Left_Encoder = new Encoder(robotSystem.BACK_LEFT_ENCODER_A, robotSystem.BACK_LEFT_ENCODER_B);
	        Back_Left_Encoder.setDistancePerPulse(.1);
	        //YAW RATE (GYRO)
	        roboGyro = new Gyro(robotSystem.GYRO);
	        
	        //CONTROLLER
	        
	        //PIDGyro = new PIDController(Kp,Ki,Kd,roboGyro,PIDGyroOut);
	        
	    }
	    
	    //drive modes
	    private void arcade(){
	    	
	    	myrobot.arcadeDrive(leftStick);
	    	
	    		
	    	
	    }

	    private void mecanum_fieldOriented(){
	    	myrobot.mecanumDrive_Cartesian(leftStick.getX(), leftStick.getY(), rightStick.getX(), roboGyro.getAngle());
	    }
	    public void driveStraight(double heading,double speed) {
	    	double error = heading - roboGyro.getAngle();
	    	double turnRate = error * Kp;
	    	double maxturnRate= .75;
	    	if (turnRate >maxturnRate)
	    		turnRate=maxturnRate;
	    	else if (turnRate < -maxturnRate)
	    		turnRate = -maxturnRate;
	    	myrobot.arcadeDrive(speed,turnRate);
	    	SmartDashboard.putNumber("Gyro", roboGyro.getAngle());
	    }
	    
	    public void driveMecanum(double heading,double speed, double direction) {
	    	double error = heading - roboGyro.getAngle();
	    	double turnRate = error * Kp;
	    	double maxturnRate= .75;
	    	if (turnRate >maxturnRate)
	    		turnRate=maxturnRate;
	    	else if (turnRate < -maxturnRate)
	    		turnRate = -maxturnRate;
	    	myrobot.mecanumDrive_Polar(speed, direction, turnRate);
	    	SmartDashboard.putNumber("Gyro", roboGyro.getAngle());
	    }
	    public void GyroReset() {
	    	roboGyro.reset(); 
	    	
	    }
	    //drive selector

	    public void run(){
	    	arcade();
	    	if (leftStick.getRawButton(1) == true)
	    		roboGyro.reset();
	    	//System.out.println("gyro = " + roboGyro.getAngle());
	    	
	    	//System.out.println("distance = " + Front_Right_Encoder.get() + " Rate = " + Front_Right_Encoder.getRate());
	    	SmartDashboard.putNumber("Gyro", roboGyro.getAngle());
	    	SmartDashboard.putNumber("Encoder Distance",Front_Right_Encoder.get());
	    	SmartDashboard.putNumber("Encoder Rate",Front_Right_Encoder.getRate());
		}
	    

}    


