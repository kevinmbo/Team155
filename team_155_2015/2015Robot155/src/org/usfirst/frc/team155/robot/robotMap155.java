package org.usfirst.frc.team155.robot;

/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */


import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 * @author kevin
 */
public class robotMap155 {
	
    //The constants to define robot connections
    //PWMs
	public final int DRIVE_LEFT_FRONT = 0;
    public final int DRIVE_LEFT_REAR = 1;
    public final int DRIVE_RIGHT_FRONT = 2;
    public final int DRIVE_RIGHT_REAR = 3;
    public final int LIFT_MOTOR = 4;
    public final int PWM5 = 5;
    public final int PWM6 = 6;
    public final int PWM7 = 7;
    public final int PWM8 = 8;
    public final int PWM9 = 9;
    //solenoids
    public final int GRIPPER_SIDE_A = 0;
    public final int GRIPPER_SIDE_B = 1;
    public final int SOL_2 = 2;
    public final int SOL_3 = 3;
    public final int SOL_4 = 4;
    public final int SOL_5 = 5;
    public final int SOL_6 = 6;
    public final int SOL_7 = 7;
    
    //digital I/O
    public final int LOW_LIMIT = 0;
    public final int HIGH_LIMIT = 1;
    public final int DIG_IO_2 = 2;
    public final int DIG_IO_3 = 3;
    public final int DIG_IO_4 = 4;
    public final int DIG_IO_5 = 5;
    public final int DIG_IO_6 = 6;
    public final int DIG_IO_7 = 7;
    public final int DIG_IO_8 = 8;
    public final int DIG_IO_9 = 9;
    
    //MXP digitals
    public final int FRONT_LEFT_ENCODER_A   = 10;
    public final int FRONT_LEFT_ENCODER_B   = 11;
    public final int REAR_LEFT_ENCODER_A    = 12;
    public final int REAR_LEFT_ENCODER_B    = 13;
    public final int FRONT_RIGHT_ENCODER_A  = 14;
    public final int FRONT_RIGHT_ENCODER_B  = 15;
    public final int REAR_RIGHT_ENCODER_A   = 16;
    public final int REAR_RIGHT_ENCODER_B   = 17;
    public final int DIG_MXP_IO_18           = 18;
    public final int DIG_MXP_IO_19           = 19;
    public final int DIG_MXP_IO_20          = 20;
    public final int DIG_MXP_IO_21          = 21;
    public final int DIG_MXP_IO_22          = 22;
    public final int DIG_MXP_IO_23          = 23;
    public final int DIG_MXP_IO_24          = 24;
    public final int DIG_MXP_IO_25          = 25;
    
    //relays
    public final int RELAY_0 = 0;
    public final int RELAY_1 = 1;
    public final int RELAY_2 = 2;
    public final int RELAY_3 = 3;

    //ANALOGS
    public final int GYRO = 0;
    public final int ANALOG_1 = 1;
    public final int ANALOG_2 = 2;
    public final int ANALOG_3 = 3;


    //declare inputs to be used in multiple objects here
    //	call as robotSystem.dig1.get();
    DigitalInput dig1;


    public robotMap155() {
      // dig1 = new DigitalInput(DIG_IO_0);


    }

    public void sensorOutput() {

        SmartDashboard.putBoolean("Ball Switch = ", dig1.get());

    }
}

