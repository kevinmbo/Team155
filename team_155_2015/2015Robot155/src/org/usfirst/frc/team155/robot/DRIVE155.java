package org.usfirst.frc.team155.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj.Buttons.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DRIVE155 {
		robotMap155 robotSystem;
		SmartDashboard sDash;
		//Vision155 vision;
		
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
	    double camKp=.01;
	    
	    public DRIVE155(Joystick left, Joystick right, robotMap155 robot) {
	        robotSystem = robot;
	        sDash = new SmartDashboard();
	        //vision = new Vision155();
	        
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
	        myrobot.setInvertedMotor(RobotDrive.MotorType.kFrontRight, false);
	        myrobot.setInvertedMotor(RobotDrive.MotorType.kRearLeft, true);
	        myrobot.setInvertedMotor(RobotDrive.MotorType.kRearRight, false);
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
	   // private void arcade(){
	    	
	    	//myrobot.arcadeDrive(leftStick);
	    	
	    		
	    	
	    //}

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
	    
	    public void driveMecanum(double heading, double speed, double direction) {
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
	    
	    public void centerYellowTote(double goalposition, double speed, double toteposition){
	    	double error = goalposition - toteposition;
	    	double slideRate = error * camKp;
	    	double maxslideRate= .75;
	    	if (slideRate >maxslideRate)
	    		slideRate=maxslideRate;
	    	else if (slideRate < -maxslideRate)
	    		slideRate = -maxslideRate;
	    	Timer.delay(0.005);
	    	myrobot.mecanumDrive_Cartesian(slideRate, speed, 0, roboGyro.getAngle());
	    	SmartDashboard.putNumber("toteposition=", toteposition);
	    	SmartDashboard.putNumber("error=",error);
	    }
	  
	    public void GyroReset() {
	    	roboGyro.reset(); 
	    	
	    }
	    //drive selector
	    public void mecanumstop(){
	    	myrobot.mecanumDrive_Polar(0,0,0);
	    }
	    public void run(){
	    	/*arcade();
	    	if (leftStick.getRawButton(1) == true)
	    		roboGyro.reset();
	    	//System.out.println("gyro = " + roboGyro.getAngle());
	    	
	    	//System.out.println("distance = " + Front_Right_Encoder.get() + " Rate = " + Front_Right_Encoder.getRate());
	    	SmartDashboard.putNumber("Gyro", roboGyro.getAngle());
	    	SmartDashboard.putNumber("Encoder Distance",Front_Right_Encoder.get());
	    	SmartDashboard.putNumber("Encoder Rate",Front_Right_Encoder.getRate());
		*/
	    	 mecanum_fieldOriented();
	    	if (leftStick.getRawButton(1) == true)
	    		roboGyro.reset();	
	    	if (leftStick.getRawButton(2) == true)
	    		EncoderReset();	
	    
	    }
	    
	    public double EncoderDistance(){
	    	double distance_Back_Left;
	    	double distance_Front_Left;
	    	double averageDistance;
	    	distance_Front_Left = Front_Left_Encoder.getDistance();
	    	distance_Back_Left = Back_Left_Encoder.getDistance();
	    	averageDistance = (distance_Front_Left + distance_Back_Left) / 2;
	    	SmartDashboard.putNumber("Average of left side encoder : ", averageDistance);
	    	SmartDashboard.putNumber("Back left Encoder Distance : ", Back_Left_Encoder.getDistance());
	    	SmartDashboard.putNumber("Front left Encoder Distance : ", Front_Left_Encoder.getDistance());
	    	return averageDistance;
	    }
	    public void EncoderRate(){
	    	double rate;
	    	rate = Front_Left_Encoder.getRate();
	    	SmartDashboard.putNumber("Back left Encoder Rate : ", Back_Left_Encoder.getRate());
	    	SmartDashboard.putNumber("Front left Encoder Rate: ", Front_Left_Encoder.getRate());
	    }
	    public void EncoderReset(){
	    	Front_Left_Encoder.reset();
	    	Back_Left_Encoder.reset();
	    }
	    

}    


