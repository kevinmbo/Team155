package org.usfirst.frc.team155.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.AnalogInput;
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
	    //public Talon left_front;
	    //public Talon right_front;
	    //public Talon left_back;
	    //public Talon right_back;
	    
	    public Jaguar left_front;
	    public Jaguar right_front;
	    public Jaguar left_back;
	    public Jaguar right_back;
	    
	    public RobotDrive myrobot;
	    
	    
	    //DRIVE ENCODERS
	    public Encoder Front_Right_Encoder;
	    public Encoder Front_Left_Encoder;
	    public Encoder Back_Right_Encoder;
	    public Encoder Back_Left_Encoder;
	    public PIDController Front_Right_PID;
	    public PIDController Front_Left_PID;
	    public PIDController Rear_Right_PID;
	    public PIDController Rear_Left_PID;
	    
	    
	    //YAW RATE SENSOR
	    public Gyro roboGyro;
	    public AnalogInput rangefinder;
	    
	    //CONTROLLERS
	    double Kp = .05;
        double Ki = 0;
        double Kd = 0;
        double PIDGyroOut;
	    public PIDController PIDGyro;
	    double camKp=.005;
	    
	    //Encoder Controllers
	    double Front_Right_Kp = .05;
        double Front_Right_Ki = 0;
        double Front_Right_Kd = 0;
        double Front_Left_Kp = .05;
        double Front_Left_Ki = 0;
        double Front_Left_Kd = 0;
        double Rear_Right_Kp = .05;
        double Rear_Right_Ki = 0;
        double Rear_Right_Kd = 0;
        double Rear_Left_Kp = .05;
        double Rear_Left_Ki = 0;
        double Rear_Left_Kd = 0;
	    
	    public DRIVE155(Joystick left, Joystick right, robotMap155 robot) {
	        robotSystem = robot;
	        sDash = new SmartDashboard();
	        //vision = new Vision155();
	        
	        
	        //JOYSTICKS - NEEDS MORE APPROPRIATE NAMES
	        leftStick = left;
	        rightStick = right;

	        //MOTORS
	        //left_front = new Talon(robotSystem.DRIVE_LEFT_FRONT);
	        //right_front = new Talon(robotSystem.DRIVE_RIGHT_FRONT);
	        //left_back = new Talon(robotSystem.DRIVE_LEFT_BACK);
	        //right_back = new Talon(robotSystem.DRIVE_RIGHT_BACK);
	   
	        left_front = new Jaguar(robotSystem.DRIVE_LEFT_FRONT);
	        right_front = new Jaguar(robotSystem.DRIVE_RIGHT_FRONT);
	        left_back = new Jaguar(robotSystem.DRIVE_LEFT_BACK);
	        right_back = new Jaguar(robotSystem.DRIVE_RIGHT_BACK);
	        

	        myrobot = new RobotDrive(left_front,left_back,right_front,right_back);
	        
	        myrobot.setInvertedMotor(RobotDrive.MotorType.kFrontLeft, true);
	        myrobot.setInvertedMotor(RobotDrive.MotorType.kFrontRight, false);
	        myrobot.setInvertedMotor(RobotDrive.MotorType.kRearLeft, true);
	        myrobot.setInvertedMotor(RobotDrive.MotorType.kRearRight, false);
	        //ENCODERS
	        Front_Right_Encoder = new Encoder(robotSystem.FRONT_RIGHT_ENCODER_A, robotSystem.FRONT_RIGHT_ENCODER_B);
	        Front_Right_Encoder.setDistancePerPulse(6*3.14159265389/500);
	        Front_Left_Encoder = new Encoder(robotSystem.FRONT_LEFT_ENCODER_A, robotSystem.FRONT_LEFT_ENCODER_B);
	        Front_Left_Encoder.setDistancePerPulse(6*3.14159265389/500); 
	        Back_Right_Encoder = new Encoder(robotSystem.BACK_RIGHT_ENCODER_A, robotSystem.BACK_RIGHT_ENCODER_B);
	        Back_Right_Encoder.setDistancePerPulse(6*3.14159265389/500);
	        Back_Left_Encoder = new Encoder(robotSystem.BACK_LEFT_ENCODER_A, robotSystem.BACK_LEFT_ENCODER_B);
	        Back_Left_Encoder.setDistancePerPulse(6*3.14159265389/500);
	       
	        //YAW RATE (GYRO)
	        roboGyro = new Gyro(robotSystem.GYRO);
	    	        
	        //CONTROLLER
	        Front_Right_Encoder.setPIDSourceParameter(PIDSource.PIDSourceParameter.kDistance);
	        Front_Right_PID = new PIDController(Front_Right_Kp, Front_Right_Ki, Front_Right_Kd, Front_Right_Encoder, right_front);
	        Front_Left_Encoder.setPIDSourceParameter(PIDSource.PIDSourceParameter.kDistance);
	        Front_Left_PID = new PIDController(Front_Left_Kp, Front_Left_Ki, Front_Left_Kd, Front_Left_Encoder, left_front);
	        Back_Right_Encoder.setPIDSourceParameter(PIDSource.PIDSourceParameter.kDistance);
	        Rear_Right_PID = new PIDController(Rear_Right_Kp, Rear_Right_Ki, Rear_Right_Kd, Back_Right_Encoder, right_back);
	        Back_Left_Encoder.setPIDSourceParameter(PIDSource.PIDSourceParameter.kDistance);
	        Rear_Left_PID = new PIDController(Rear_Left_Kp, Rear_Left_Ki, Rear_Left_Kd, Back_Left_Encoder, left_back);
	        Front_Left_PID.setAbsoluteTolerance(2);
	        Front_Right_PID.setAbsoluteTolerance(2);
	        Rear_Left_PID.setAbsoluteTolerance(2);
	        Rear_Right_PID.setAbsoluteTolerance(2);
	        Front_Left_PID.setOutputRange(-.5, .5);
	        Front_Right_PID.setOutputRange(-.5, .5);
	        Rear_Left_PID.setOutputRange(-.5, .5);
	        Rear_Right_PID.setOutputRange(-.5, .5);
	        
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
	    	//SmartDashboard.putNumber("Gyro", roboGyro.getAngle());
	    }
	    
	    public void driveMecanum(double heading, double speed, double direction) {
			System.out.println("In driveMecanum");
			System.out.println("heading = "+ heading);
			System.out.println("speed = "+ speed);
			System.out.println("direction = "+ direction);
			
	    	System.out.println("roboGyro.getAngle() = "+ roboGyro.getAngle());
			double error = heading - roboGyro.getAngle();
			System.out.println("error = "+ error);
			
	    	PowerDistributionPanel pdp = new PowerDistributionPanel();
	    	double turnRate = error * Kp;
			System.out.println("turnRate = "+ turnRate);
			
			pdp.clearStickyFaults(); //clear the brownout fallouts from last run, if there are any
			
	    	double maxturnRate= .75;
	    	
	    	if (turnRate >maxturnRate)
	    		turnRate=maxturnRate;
	    	else if (turnRate < -maxturnRate)
	    		turnRate = -maxturnRate;
	    	System.out.println("turnRate(after max compare) = "+ turnRate);
	    	System.out.println("Going to mecanumDrive_Polar");
			System.out.println("speed = "+ speed);
			System.out.println("direction = "+ direction);
	    	System.out.println("turnRate(after max compare) = "+ turnRate);

	    	myrobot.mecanumDrive_Polar(speed, direction, turnRate);
	    	//SmartDashboard.putNumber("Gyro", roboGyro.getAngle());
	    	//SmartDashboard.putNumber("Motor1 current", pdp.getCurrent(12));
	    }
	    
	    public void centerYellowTote(double goalposition, double speed, double toteposition){
	    	
			System.out.println("In centerYellowTote");
			System.out.println("goalposition = " + goalposition);
			System.out.println("speed = " + speed);
			System.out.println("toteposition = " + toteposition);
			
	    	double error = goalposition - toteposition;
			System.out.println("error(= goalposition - toteposition) = " + error);
			
			System.out.println("camKp = " + camKp);
			double slideRate = -error * camKp;
			System.out.println("slideRate(= -error * camKp) = " + slideRate);
			
	    	double maxslideRate= .5;
	    	//double minslideRate = .1;
	    	
	    	if (slideRate > maxslideRate)
	    		slideRate= maxslideRate;
	    	else if (slideRate < -maxslideRate)
	    		slideRate = -maxslideRate;
	    	
			System.out.println("slideRate(adj for maxslideRate) = " + slideRate);
			
	    	//if ((slideRate < minslideRate)&&(slideRate > -minslideRate))
	    	//	slideRate = 0;
	    	//Timer.delay(0.005);
	    	
			System.out.println("Going to mecanumDrive_Cartesian");
			System.out.println("x = " + slideRate);
			System.out.println("y = 0");
			System.out.println("speed = " + speed);
			System.out.println("GyroAngle = " + roboGyro.getAngle());
			
			myrobot.mecanumDrive_Cartesian(slideRate, speed, 0, roboGyro.getAngle());
			
	    	SmartDashboard.putNumber("toteposition=", toteposition);
	    	SmartDashboard.putNumber("error=",error);
	    	SmartDashboard.putNumber("sliderate = ", slideRate);
	    }
	    
	    public double getGyro() {
		  return roboGyro.getAngle();
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
	    	PowerDistributionPanel pdp = new PowerDistributionPanel();
	    	//SmartDashboard.putNumber(key, value);
	    	SmartDashboard.putNumber("Motor0 current", pdp.getCurrent(0));
	    	SmartDashboard.putNumber("Motor1 current", pdp.getCurrent(1));
	    	SmartDashboard.putNumber("Motor12 current", pdp.getCurrent(12));
	    	SmartDashboard.putNumber("Motor13 current", pdp.getCurrent(13));	    	
	    	SmartDashboard.putNumber("pdp current",pdp.getVoltage());
	    	
	    	mecanum_fieldOriented();
	    	if (leftStick.getRawButton(1) == true)
	    		//PIDEnable();
	    		roboGyro.reset();	
	    	if (leftStick.getRawButton(2) == true)
	    		EncoderReset();
	    	if (leftStick.getRawButton(7) == true)
	    		PIDDisable();
	    	//DriveStraightDistance(36);
	    }
	    
	    public double EncoderDistance(){
	    	double distance_Back_Left;
	    	double distance_Front_Left;
	    	double distance_Back_Right;
	    	double distance_Front_Right;
	    	double averageDistance;
	    	distance_Front_Left = Front_Left_Encoder.getDistance();
	    	distance_Back_Left = Back_Left_Encoder.getDistance();
	    	distance_Front_Right = Front_Left_Encoder.getDistance();
	    	distance_Back_Right = Back_Left_Encoder.getDistance();
	    	//averageDistance = (distance_Front_Left + distance_Back_Left + distance_Front_Right + distance_Back_Right) / 4;
	    	averageDistance = (distance_Front_Left + distance_Back_Left) / 2;
	    	SmartDashboard.putNumber("Average of left side encoder : ", averageDistance);
	    	SmartDashboard.putNumber("Back left Encoder Distance : ", Back_Left_Encoder.getDistance());
	    	SmartDashboard.putNumber("Front left Encoder Distance : ", Front_Left_Encoder.getDistance());
	    	SmartDashboard.putNumber("Back Right Encoder Distance : ", Back_Right_Encoder.getDistance());
	    	SmartDashboard.putNumber("Front Right Encoder Distance : ", Front_Right_Encoder.getDistance());
	    	
	    	return averageDistance;
	    }
	    public void EncoderRate(){
	    	double rate;
	    	System.out.println("in EncoderRate");

	    	rate = Front_Left_Encoder.getRate();
	    	
			System.out.println("rate = " + rate);
	    	SmartDashboard.putNumber("Back left Encoder Rate : ", Back_Left_Encoder.getRate());
	    	SmartDashboard.putNumber("Front left Encoder Rate: ", Front_Left_Encoder.getRate());
	    	SmartDashboard.putNumber("Back Right Encoder Rate : ", Back_Right_Encoder.getRate());
	    	SmartDashboard.putNumber("Front Right Encoder Rate: ", Front_Right_Encoder.getRate());
	    }
	    public void EncoderReset(){
	    	
	    	Front_Left_Encoder.reset();
	    	Back_Left_Encoder.reset();
	    	Front_Right_Encoder.reset();
	    	Back_Right_Encoder.reset();
	    	
	    }
	    public void RangeFinderDistance(){
	    	
	    }
	    
	    public boolean DriveStraightDistance(double distance) {
			System.out.println("Drive straight for 84 inches");
	    	SmartDashboard.putNumber("Back left Encoder Distance : ", Back_Left_Encoder.getDistance());
	    	SmartDashboard.putNumber("Front left Encoder Distance : ", Front_Left_Encoder.getDistance());
	    	SmartDashboard.putNumber("Back Right Encoder Distance : ", Back_Right_Encoder.getDistance());
	    	SmartDashboard.putNumber("Front Right Encoder Distance : ", Front_Right_Encoder.getDistance());    	
	    	Front_Right_PID.setSetpoint(distance);
	    	Front_Left_PID.setSetpoint(-distance);
	    	Rear_Right_PID.setSetpoint(distance);
	    	Rear_Left_PID.setSetpoint(-distance);
	    	return Front_Left_PID.onTarget();
	    }

	    public boolean DriveSideDistance(double distance) {
	    	double adjusted;
	    	double scale = 1;
	    	adjusted = scale * distance;
	    	Front_Right_PID.setSetpoint(-adjusted);
	    	Front_Left_PID.setSetpoint(-adjusted);
	    	Rear_Right_PID.setSetpoint(adjusted);
	    	Rear_Left_PID.setSetpoint(adjusted);
	    	return Front_Left_PID.onTarget();
	    }
	    
	    public void PIDEnable(){
	    Front_Left_PID.enable();
	    Front_Right_PID.enable();
	    Rear_Left_PID.enable();
	    Rear_Right_PID.enable();
	    }
	    public void PIDDisable(){
	    	Front_Left_PID.disable();
		    Front_Right_PID.disable();
		    Rear_Left_PID.disable();
		    Rear_Right_PID.disable();
		    Front_Left_PID.reset();
		    Front_Right_PID.reset();
		    Rear_Left_PID.reset();
		    Rear_Right_PID.reset();
	    }
	    
}    


