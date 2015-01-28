package org.usfirst.frc.team155.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PowerDistributionPanel;

import java.util.logging.Logger;

import org.ejml.data.DenseMatrix64F;
import org.ejml.UtilEjml;
import org.ejml.ops.CommonOps;
import org.ejml.ops.MatrixIO;

import static org.ejml.ops.CommonOps.*;

public class mecanum155 implements Runnable {
	private static Logger logFile = Logger.getLogger("mecanum_controller.log");

	robotMap155 robot;

	// MOTORS
	private Talon leftFront;
	private Talon rightFront;
	private Talon leftRear;
	private Talon rightRear;

	// encoders
	public Encoder leftFrontEncoder; // the encoders
	public Encoder rightFrontEncoder;
	public Encoder leftRearEncoder;
	public Encoder rightRearEncoder;

	private int m_samplesAverage = 10;
	private int m_ticks = 250;

	// yaw rate
	public Gyro roboGyro;

	// joystick axis values
	double x_J1;
	double y_J1;
	double x_J2;
	double y_J2;

	// PDP
	private PowerDistributionPanel pdp;

	// for the state space stuff

	private double m_currLF; // currents
	private double m_currLR;
	private double m_currRF;
	private double m_currRR;

	private double m_Vbatt; // battery voltage

	private double m_velLF; // velocities
	private double m_velLR;
	private double m_velRF;
	private double m_velRR;

	/*
	 * KALMAN FILTER MATRICES
	 */
	// base matrices
	DenseMatrix64F F;
	DenseMatrix64F Q;
	DenseMatrix64F R;

	DenseMatrix64F B;
	DenseMatrix64F H;

	DenseMatrix64F P;
	DenseMatrix64F u;
	DenseMatrix64F z;
	DenseMatrix64F K;
	DenseMatrix64F y;
	DenseMatrix64F xEstimate;

	DenseMatrix64F I;
	// internalized temporary matrices
	DenseMatrix64F addTerm1;
	DenseMatrix64F addTerm2;
	DenseMatrix64F addTerm3;
	DenseMatrix64F addTerm4;
	DenseMatrix64F addTerm5;
	DenseMatrix64F prodTerm1;
	DenseMatrix64F prodTerm2;
	DenseMatrix64F prodTerm3;
	DenseMatrix64F partialProd;
	DenseMatrix64F Ftran;
	DenseMatrix64F Htran;
	DenseMatrix64F subTerm1;
	DenseMatrix64F subTerm2;

	DenseMatrix64F invertPre;
	DenseMatrix64F invertPost;

	// for the controller
	DenseMatrix64F output;

	double maximum;
	double minimum;

	// configuration parameters
	private boolean m_driveMode;
	private int m_mode;
	private final int LOGGING = 0;
	private final int MECANUM_RUN = 1;
	private final int TANK_DRIVE = 2;
	private final int SYSTEM_IDENTIFICATION = 3;
	private final int ARCADE_DRIVE = 4;
	private final int NULL = 5;

	double update_rate_hz = 80;

	// do something similar to this?
	// why is it does like shown below?
	// does it matter?
	public static class MotorType {

		/**
		 * The integer value representing this enumeration
		 */
		public final int value;
		static final int kFrontLeft_val = 0;
		static final int kFrontRight_val = 1;
		static final int kRearLeft_val = 2;
		static final int kRearRight_val = 3;
		/**
		 * motortype: front left
		 */
		public static final MotorType kFrontLeft = new MotorType(kFrontLeft_val);
		/**
		 * motortype: front right
		 */
		public static final MotorType kFrontRight = new MotorType(
				kFrontRight_val);
		/**
		 * motortype: rear left
		 */
		public static final MotorType kRearLeft = new MotorType(kRearLeft_val);
		/**
		 * motortype: rear right
		 */
		public static final MotorType kRearRight = new MotorType(kRearRight_val);

		private MotorType(int value) {
			this.value = value;
		}
	}

	// threading...
	Thread m_thread;
	double startTime;
	double endCalcTime;
	double deltaTime;
	int loopCount;

	mecanum155(robotMap155 robot) {
		this.robot = robot;
		pdp = new PowerDistributionPanel();

		// motors
		leftFront = new Talon(robot.DRIVE_LEFT_FRONT);
		rightFront = new Talon(robot.DRIVE_RIGHT_FRONT);
		leftRear = new Talon(robot.DRIVE_LEFT_REAR);
		rightRear = new Talon(robot.DRIVE_RIGHT_REAR);

		leftFront.enableDeadbandElimination(true);
		rightFront.enableDeadbandElimination(true);
		leftRear.enableDeadbandElimination(true);
		rightRear.enableDeadbandElimination(true);

		// encoders
		// construct the encoder channels, don't reverse, count all edges,
		// average over some time period
		leftFrontEncoder = new Encoder(robot.FRONT_LEFT_ENCODER_A,
				robot.FRONT_LEFT_ENCODER_B, false, Encoder.EncodingType.k4X);
		rightFrontEncoder = new Encoder(robot.FRONT_RIGHT_ENCODER_A,
				robot.FRONT_RIGHT_ENCODER_B, false, Encoder.EncodingType.k4X);
		leftRearEncoder = new Encoder(robot.REAR_LEFT_ENCODER_A,
				robot.REAR_LEFT_ENCODER_B, false, Encoder.EncodingType.k4X);
		rightRearEncoder = new Encoder(robot.REAR_RIGHT_ENCODER_A,
				robot.REAR_RIGHT_ENCODER_B, false, Encoder.EncodingType.k4X);

		leftFrontEncoder.setSamplesToAverage(m_samplesAverage); // average them
																// a bit
		rightFrontEncoder.setSamplesToAverage(m_samplesAverage);
		leftRearEncoder.setSamplesToAverage(m_samplesAverage);
		rightRearEncoder.setSamplesToAverage(m_samplesAverage);

		leftFrontEncoder.setDistancePerPulse(1 / m_ticks); // 1 distance unit =
															// 1 revolution, may
															// want to change to
															// radians
		rightFrontEncoder.setDistancePerPulse(1 / m_ticks);
		leftRearEncoder.setDistancePerPulse(1 / m_ticks);
		rightRearEncoder.setDistancePerPulse(1 / m_ticks);

		// yaw rate
		roboGyro = new Gyro(robot.GYRO);

		// threading
		m_thread = new Thread(this);
		m_thread.start();
		loopCount = 0;

		/*
		 * 
		 * SETTING UP THE STATE SPACE EQUATION FOR PRE-COMPILING
		 */
		// Kalman filter -- not verified for accuracy

		// i really should make this parameterizable.....

		/*
		 * KALMAN FILTER MATRICES
		 */
		// base matrices
		final int states = 8;
		final int inputs = 4;
		final int outputs = 8;

		F = new DenseMatrix64F(states,states);		//'A" matrix
		Q = new DenseMatrix64F(states,states);		//noise covariance of process noise
		R = new DenseMatrix64F(outputs,outputs);	//noise ocvariance of measeurement noise

		B = new DenseMatrix64F(states,inputs);		//'B' matrix
		H = new DenseMatrix64F(outputs,states);		//'C' matrix

		P = new DenseMatrix64F(states,states);		//posteriori error covariance matrix
		u = new DenseMatrix64F(inputs,1);			//input matrix
		z = new DenseMatrix64F(outputs,1);			//observation or measurement matrix
		K = new DenseMatrix64F(states,outputs);		//kalman gains matrix
		y = new DenseMatrix64F(outputs,1);			//error matrix
		xEstimate = new DenseMatrix64F(states, 1);	//the estimated state matrix

		I = new DenseMatrix64F(states,states);
		
		//initialize the above matrices
		I=identity(states);
		
		// internalized temporary matrices
		addTerm1 = new DenseMatrix64F(states,1);
		addTerm2 = new DenseMatrix64F(states,1);
		addTerm3 = new DenseMatrix64F(states,states);
		addTerm4 = new DenseMatrix64F(outputs,outputs);
		addTerm5 = new DenseMatrix64F(states,1);
		prodTerm1 = new DenseMatrix64F(outputs,states);
		prodTerm2 = new DenseMatrix64F(outputs,states);
		prodTerm3 = new DenseMatrix64F(states,outputs);
		partialProd = new DenseMatrix64F(states,states);
		Ftran = new DenseMatrix64F(states,states);
		Htran = new DenseMatrix64F(states,outputs);
		subTerm1 = new DenseMatrix64F(outputs,1);
		subTerm2 = new DenseMatrix64F(states,states);

		invertPre = new DenseMatrix64F(outputs,outputs);
		invertPost = new DenseMatrix64F(outputs,outputs);



	}

	public void run() {

		while (true) {
			try {
				startTime = Timer.getFPGATimestamp();
				loopCount++;

				// get the wheel speed
				m_velLF = leftFrontEncoder.getRate();
				m_velLR = leftRearEncoder.getRate();
				m_velRF = rightFrontEncoder.getRate();
				m_velRR = rightRearEncoder.getRate();
				// get the currents
				m_currLF = pdp.getCurrent(robot.DRIVE_LEFT_FRONT);
				m_currLR = pdp.getCurrent(robot.DRIVE_LEFT_REAR);
				m_currRF = pdp.getCurrent(robot.DRIVE_RIGHT_FRONT);
				m_currRR = pdp.getCurrent(robot.DRIVE_RIGHT_REAR);
				// get battery voltage
				m_Vbatt = pdp.getVoltage();

				switch (m_mode) {
				case LOGGING:
					// log wheel velocities
					logFile.info(startTime + "wheelSpeed(" + loopCount
							+ ",1:1:4)=[" + m_velLF + ", " + m_velLR + ", "
							+ m_velRF + ", " + m_velRR + "];");
					// log joystick values
					logFile.info(startTime + "joyStick(" + loopCount
							+ ",1:1:4)=[" + x_J1 + ", " + y_J1 + ", " + x_J2
							+ ", " + y_J2 + "];");
					// log battery voltage
					logFile.info(startTime + "vbatt(" + loopCount + ",1)=["
							+ m_Vbatt + "];");
					break;
				case MECANUM_RUN:
					/*
					 * kalman observer for the drive
					 */

					// prepping up the input (u) and output measurement(z) for
					// the kalman filter
					double[] z_prep = new double[] { m_velLF, m_currLF,
							m_velLR, m_currLR, m_velRF, m_currRF, m_velRR,
							m_currRR };
					double[] u_prep = new double[] { leftFront.get(),
							leftRear.get(), rightFront.get(), rightRear.get() };

					u.setData(u_prep); // is this right?
					z.setData(z_prep); // is this right

					scale(m_Vbatt, u);

					/*
					 * RUN THE KALMAN FILTER
					 */
					xEstimate = kalmanFilter(u, z);

					/*
					 * calculate drive wheel speeds based on user input
					 */

					/*
					 * run the controller based on user input must take into
					 * account for the limits of [-1,1] output scaling to motors
					 * and Vbatt
					 */

					divide(m_Vbatt, output); // normalize to present battery
												// voltage

					maximum = elementMaxAbs(output); // find the largest value
					minimum = elementMinAbs(output);

					if (minimum > 1) {
						divide(maximum, output);
						logFile.info("WARNING: AT "
								+ startTime
								+ " THERE IS IN SUFFICIENT BATTERY VOLTAGE TO PRODUCE THE NEEDED RPM ON ALL MOTORS");
					} else if (maximum > 1) {
						divide(maximum, output);
						logFile.info("WARNING: AT " + startTime
								+ " MAXIMUM WAS OVER 1 FOR SOME MOTORS");
					}

					leftFront.set(output.data[0]); // am i doing this right? are
													// the index all correct?
					leftRear.set(output.data[1]);
					rightFront.set(output.data[2]);
					rightRear.set(output.data[3]);

					break;
				case TANK_DRIVE:
					leftFront.set(y_J1);
					rightFront.set(y_J2);
					leftRear.set(y_J1);
					rightRear.set(y_J2);
					// log wheel velocities
					logFile.info(startTime + "wheelSpeed(" + loopCount
							+ ",1:1:4)=[" + m_velLF + ", " + m_velLR + ", "
							+ m_velRF + ", " + m_velRR + "];");
					// log joystick values
					logFile.info(startTime + "joyStick(" + loopCount
							+ ",1:1:4)=[" + x_J1 + ", " + y_J1 + ", " + x_J2
							+ ", " + y_J2 + "];");
					// log output to motors
					logFile.info(startTime + "motorValues(" + loopCount
							+ ",1:1:4)=[" + leftFront.get() + ", "
							+ leftRear.get() + ", " + rightFront.get() + ", "
							+ rightRear.get() + "];");
					break;
				case ARCADE_DRIVE:
					leftFront.set(y_J1 + x_J1);
					rightFront.set(y_J1 - x_J1);
					leftRear.set(y_J1 + x_J1);
					rightRear.set(y_J1 - x_J1);
					// log wheel velocities
					logFile.info(startTime + "wheelSpeed(" + loopCount
							+ ",1:1:4)=[" + m_velLF + ", " + m_velLR + ", "
							+ m_velRF + ", " + m_velRR + "];");
					// log joystick values
					logFile.info(startTime + "joyStick(" + loopCount
							+ ",1:1:4)=[" + x_J1 + ", " + y_J1 + ", " + x_J2
							+ ", " + y_J2 + "];");
					// log output to motors
					logFile.info(startTime + "motorValues(" + loopCount
							+ ",1:1:4)=[" + leftFront.get() + ", "
							+ leftRear.get() + ", " + rightFront.get() + ", "
							+ rightRear.get() + "];");
					// log current levels from motors
					logFile.info(startTime + "motorCurrent(" + loopCount
							+ ",1:1:4)=[" + m_currLF + ", " + m_currLR + ", "
							+ m_currRF + ", " + m_currRR + "];");
					break;
				case SYSTEM_IDENTIFICATION:
					//run WPI's mecanum code
					
					
					
					// log wheel velocities
					logFile.info(startTime + "wheelSpeed(" + loopCount
							+ ",1:1:4)=[" + m_velLF + ", " + m_velLR + ", "
							+ m_velRF + ", " + m_velRR + "];");
					// log joystick values
					logFile.info(startTime + "joyStick(" + loopCount
							+ ",1:1:4)=[" + x_J1 + ", " + y_J1 + ", " + x_J2
							+ ", " + y_J2 + "];");
					// log output to motors
					logFile.info(startTime + "motorValues(" + loopCount
							+ ",1:1:4)=[" + leftFront.get() + ", "
							+ leftRear.get() + ", " + rightFront.get() + ", "
							+ rightRear.get() + "];");
					// log current levels from motors
					logFile.info(startTime + "motorCurrent(" + loopCount
							+ ",1:1:4)=[" + m_currLF + ", " + m_currLR + ", "
							+ m_currRF + ", " + m_currRR + "];");
					break;
				case NULL:
					break;

				}

				endCalcTime = Timer.getFPGATimestamp();
				deltaTime = endCalcTime - startTime;
				logFile.info("compute time: " + deltaTime + "percentage load: "
						+ deltaTime * update_rate_hz * 100);
				Timer.delay(1.0 / update_rate_hz - deltaTime);

			} catch (RuntimeException ex) {
				// This exception typically indicates a Timeout
				ex.printStackTrace();
			}
		}

	}

	// what was the purpose of this?
	public void setDrive(double x_J1, double y_J1, double x_J2, double y_J2) {

		// just set values
		this.x_J1 = x_J1;
		this.y_J1 = y_J1;
		this.x_J2 = x_J2;
		this.y_J2 = y_J2;

	}

	private DenseMatrix64F kalmanFilter(DenseMatrix64F u, DenseMatrix64F z) {
		/*
		 * x - state matrix 8x1 F - estimated A matrix H - estimated C matrix? P
		 * - some estimate matrix K - kalman gains
		 * 
		 * I - identity matrix
		 * 
		 * Q - noise matrix R - noise matrix
		 * 
		 * 
		 * z - the output observation y - error?
		 */
		boolean successTest;

		/*
		 * Predicted (a priori) state estimatex = F*x+B*u"
		 */
		mult(F, xEstimate, addTerm1);
		mult(B, u, addTerm2);
		add(addTerm1, addTerm2, xEstimate);

		/*
		 * Predicted (a prior) estimate covariance * P=F*P*F'+Q
		 */
		mult(F, P, partialProd);
		transpose(F, Ftran);
		mult(partialProd, Ftran, addTerm3);
		add(addTerm3, Q, P);

		/*
		 * innovation or measurement residual y=z-H*x
		 */

		mult(H, xEstimate, subTerm1);
		subtract(z, subTerm1, y);

		/*
		 * innovation (or residual) covariance with optimal Kalman gain
		 * calcuation K=P*H'*inv(H*P*H'+R)
		 */

		transpose(H, Htran);

		mult(H, P, prodTerm1);
		mult(prodTerm1, Htran, addTerm4);
		add(addTerm4, R, invertPre);

		successTest = invert(invertPre, invertPost);

		if (!successTest) // need to handle this better as it'll mess up the
							// kalman observer probably
			logFile.fine("SINGULAR MATRIX");

		mult(P, Htran, prodTerm3);
		mult(prodTerm3, invertPost, K);

		/*
		 * updated (a posteriori) state estimate x=x+K*y
		 */

		mult(K, y, addTerm5);
		add(xEstimate, addTerm5, xEstimate);

		/*
		 * updated (a posteriori estimate covariance P=P-K*(H*P)
		 */
		mult(H, P, prodTerm2);
		mult(K, prodTerm2, subTerm2);
		subtract(P, subTerm2, P);

		// updateP = kalmanEQ.compile("P = (I-K*H)*P*(I-K*H)'+K*R*K'");
		// //Updated (a posteriori) estimate covariance. This the joseph
		// form..... for non optimal K

		return xEstimate; // return the estimate state vector

	}

	/*
	 * true for Field Oriented Control (FOC) false for Robot Oriented Control
	 * (ROC)
	 */
	public void setDriveModeFOC(boolean mode) {
		m_driveMode = mode;
	}

	/*
	 * the counter part to above
	 */
	public boolean getDriveModeFOC() {
		return m_driveMode;
	}

	/*
	 * set and get the modes of operation
	 */
	public void setOperationMode(int mode) {
		m_mode = mode;

		switch (m_mode) {
		case LOGGING:
			logFile.info(startTime + "switching to logging only mode");
			break;
		case MECANUM_RUN:
			logFile.info(startTime + "switching to mecanum ");
			break;
		case TANK_DRIVE:
			logFile.info(startTime + "switching to tankdrive mode");
			break;
		case SYSTEM_IDENTIFICATION:
			logFile.info(startTime + "switching to system identification mode");
			break;
		case ARCADE_DRIVE:
			logFile.info(startTime + "switching to arcade drive mode");
			break;
		case NULL:
			logFile.info(startTime + "switching to do the do nothing mode");
			break;
		}
	}

	public int getOperationMode() {
		return m_mode;

	}

}
