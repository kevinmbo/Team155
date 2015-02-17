
package org.usfirst.frc.team155.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
    	
    }
    
    
    public void autonomousInit() {
    	robotDrive.GyroReset();
    	//robotDrive.PIDEnable();
    	Auto155.state= Auto155.START2;
    	//robotDrive.EncoderReset();
    	//CameraThread.run();
    	Auto155.BOX_COUNTER = 0;
    }
    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
    	Auto155.autoLine();
    	//robotVision.run();
    	//robotDrive.DriveSideDistance(84);
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
