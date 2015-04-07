package org.usfirst.frc.team155.robot;

/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */


import edu.wpi.first.wpilibj.DigitalInput;


/**
 *
 * @author kevin
 */
public class robotMap155 {

    //The constants to define robot connections
    //PWMs
	public final int DRIVE_LEFT_FRONT = 0;
    public final int DRIVE_LEFT_BACK = 1;
    public final int DRIVE_RIGHT_FRONT = 2;
    public final int DRIVE_RIGHT_BACK = 3;
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

    public final int LIFT_ENCODER_A = 0;
    public final int LIFT_ENCODER_B = 1;
    public final int FRONT_RIGHT_ENCODER_A = 2;
    public final int FRONT_RIGHT_ENCODER_B = 3;
    public final int FRONT_LEFT_ENCODER_A = 4;
    public final int FRONT_LEFT_ENCODER_B = 5;
    public final int BACK_RIGHT_ENCODER_A = 7;// this was 7
    public final int BACK_RIGHT_ENCODER_B = 6;
    public final int BACK_LEFT_ENCODER_A = 8;//8
    public final int BACK_LEFT_ENCODER_B = 9;//9
    public final int TOTE_SWITCH = 7;// 10
    public final int LOW_LIMIT = 8;// 11
    public final int HIGH_LIMIT = 9;// 12
    
    //buttons
    public final int GYRO_RESET = 1;
    public final int ENCODER_RESET = 2;
    public final int LEFTSTICK_3 = 3;
    public final int LEFTSTICK_4 = 4;
    public final int LEFTSTICK_5 = 5;
    public final int LEFTSTICK_6 = 6;
    public final int PID_DISABLE = 7;
    public final int LEFTSTICK_8 = 8;
    public final int LEFTSTICK_9 = 9;
    public final int LEFTSTICK_10 = 10;
    public final int LEFTSTICK_11 = 11;
    public final int SUCK_TOTE = 1;
    public final int SPIT_TOTE = 2;
    public final int THREE_QUARTER_SPEED = 3;
    public final int FULL_SPEED = 4;
    public final int HALF_SPEED = 5;
    public final int RIGHTSTICK_6 = 6;
    public final int RIGHTSTICK_7 = 7;
    public final int RIGHTSTICK_8 = 8;
    public final int RIGHTSTICK_9 = 9;
    public final int TOTE_ARM_UP = 10;
    public final int TOTE_ARM_DOWN = 11;
    public final int BOUNCE = 1;  
    public final int ARMSTICK_2 = 2;
    public final int ARMSTICK_3 = 3;
    public final int ARMSTICK_4 = 4;
    public final int ARMSTICK_5 = 5;
    public final int ARMSTICK_6 = 6;
    public final int ARMSTICK_7 = 7;
    public final int ARMSTICK_8 = 8;
    public final int ARMSTICK_9 = 9;
    public final int ARMSTICK_10 = 10;
    public final int ARMSTICK_11 = 11;
    
    //relays
    public final int RELAY_0 = 0;
    public final int RELAY_1 = 1;
    public final int RELAY_2 = 2;
    public final int RELAY_3 = 3;

    //ANALOGS
    public final int GYRO = 0;
    public final int RANGE_FINDER = 1;
    public final int ANALOG_2 = 2;
    public final int ANALOG_3 = 3;
    

    //declare inputs to be used in multiple objects here
    //	call as robotSystem.dig1.get();
    DigitalInput dig1;
    DigitalInput toteSwitch;
    


    public robotMap155() {
      // dig1 = new DigitalInput(DIG_IO_0);

        //toteSwitch = new DigitalInput(TOTE_SWITCH);

    }

   // public void sensorOutput() {

   //     SmartDashboard.putBoolean("Ball Switch = ", dig1.get());

  //  }
}

