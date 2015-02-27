
package org.usfirst.frc.team155.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
	
	robotMap155 robot;
	Lift155 robotLift;
	DRIVE155 robotDrive;
	Joystick rightStick;
	Joystick leftStick;
	Joystick liftStick;
	Autonomous Auto155;
	//Vision155 robotVision;
	CameraThread cameraThread;
	SendableChooser autoChooser;
	private int mode=1;
	private int modename=0;
	
	
    public void robotInit() {
    	rightStick = new Joystick(1);
    	leftStick = new Joystick(0);
    	liftStick = new Joystick(2);
    	robot = new robotMap155();
    	robotLift = new Lift155(robot, liftStick);
    	robotDrive = new DRIVE155(leftStick, rightStick, robot);
    	cameraThread = new CameraThread();
    	//cameraThread.setPriority(Thread.MIN_PRIORITY);
    	cameraThread.start();
    
    	//robotVision = new Vision155();
    	//Auto155 = new Autonomous(robotDrive, robotLift, robot, robotVision);
    	Auto155 = new Autonomous(robotDrive, robotLift, robot, cameraThread);
    	autoChooser = new SendableChooser();
    	autoChooser.addDefault("default, do nothing ", 1);
    	autoChooser.addObject("Push Straight", 2);
    	autoChooser.addObject("Push Sideways", 3);
    	autoChooser.addObject("Grab and Go", 4);
    	autoChooser.addObject("Go for Broke", 5);
    	
    	SmartDashboard.putData("Automodechooser", autoChooser);
    }
    
    
    public void autonomousInit() {
    	robotDrive.GyroReset();
    	//robotDrive.PIDEnable();
    	
    	//robotDrive.EncoderReset();
    	//CameraThread.run();
    	Auto155.BOX_COUNTER = 0;
    	mode = (int) autoChooser.getSelected();
    	Auto155.drivestate = 1;
    	Auto155.state = 0;
    	
    }
    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
    	
    	switch(mode){
    		case 1:
    			modename =1;
    			
    			break;
    		case 2:
    			modename =2;
    		
    			Auto155.driveToAutoZone();
    			break;	
    		case 3:
    			modename =3;
    			
    			Auto155.autoLine3();
    			break;
    		case 4:
    			modename =4;
    			Auto155.autoLine4(); 
    			break;
    		case 5:
    			modename =5;
    			Auto155.autoLine();
    			break;
    	}	
    	//Auto155.autoLine();		//code to pick up/stack 3 totes and move into scoring zone
    	//Auto155.autoLine3();  //code to push tote/barrel sideways into scoring zone 
    	//Auto155.autoLine4();  //code to pick up 1 tote, turn, and move into scoring zone
    	//Auto155.liftTest();	
    	//robotVision.run();
    	//robotDrive.DriveSideDistance(84);
    	SmartDashboard.putNumber("Mode name = ", modename);
    	SmartDashboard.putNumber("Gyro", robotDrive.roboGyro.getAngle());
    	robotDrive.EncoderRate();
    	
    }

    public void teleopInit() {
        
    }
    
    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
    		//call drive run() method
    		robotDrive.run();
    		//call lift run() method
    		robotLift.run();
    		//robotVision.run();
    		//robotDrive.EncoderDistance();
    		//robotDrive.EncoderRate();
    		Auto155.displayRangeFinder();
        	SmartDashboard.putNumber("Gyro", robotDrive.roboGyro.getAngle());
    		
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    
    }
    
}
