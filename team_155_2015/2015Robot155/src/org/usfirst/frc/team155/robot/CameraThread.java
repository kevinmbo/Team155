package org.usfirst.frc.team155.robot;

import java.util.Comparator;
import java.util.Vector;

import org.usfirst.frc.team155.robot.Vision155.ParticleReport;

import com.ni.vision.NIVision;
//import com.ni.vision.NIVision.GetParticleInfoResult;
import com.ni.vision.NIVision.Image;
import com.ni.vision.NIVision.ImageType;
import com.ni.vision.NIVision.RGBValue;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.image.ParticleAnalysisReport;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.vision.USBCamera;

public class CameraThread extends Thread {

	int session;
	// Image frame;
	// AxisCamera camera;
	//USBCamera camera;

	int inited;
	private boolean running;
	// CriteriaCollection cc;
	// ParticleFilterCriteria2[] criteria;
	// final int AREA_MINIMUM = 150;
	ParticleAnalysisReport report;
	private double horizontal;
	private double vertical;
	private boolean m_foundTote;

	CameraThread() {
		frame = NIVision.imaqCreateImage(NIVision.ImageType.IMAGE_HSL, 0);

		// open the camera at the IP address assigned. This is the IP address
		// that the camera
		// can be accessed through the web interface.
		// camera = new AxisCamera("10.1.55.11");
		//camera = new USBCamera();
		inited = 0;

		// cc = new CriteriaCollection(); // create the criteria for the
		// particle filter
		// cc.addCriteria(NIVision.MeasurementType.IMAQ_MT_AREA, AREA_MINIMUM,
		// 65535, false);
		// criteria={NIVision.MeasurementType.MT_AREA_BY_IMAGE_AREA,
		// AREA_MINIMUM, 65535, false, false};

		// create images
		// frame = NIVision.imaqCreateImage(ImageType.IMAGE_RGB, 0);
		binaryFrame = NIVision.imaqCreateImage(ImageType.IMAGE_U8, 0);
		criteria[0] = new NIVision.ParticleFilterCriteria2(
				NIVision.MeasurementType.MT_AREA_BY_IMAGE_AREA, AREA_MINIMUM,
				100.0, 0, 0);

		// the camera name (ex "cam0") can be found through the roborio web
		// interface
		session = NIVision.IMAQdxOpenCamera("cam0",
				NIVision.IMAQdxCameraControlMode.CameraControlModeController);
		NIVision.IMAQdxConfigureGrab(session);

		// NIVision.imaqWritePNGFile2(frame, "raw_image2", 750, colorTable, 0);

		// Put default values to SmartDashboard so fields will appear
		// SmartDashboard.putNumber("Tote hue min", TOTE_HUE_RANGE.minValue);
		// SmartDashboard.putNumber("Tote hue max", TOTE_HUE_RANGE.maxValue);
		// SmartDashboard.putNumber("Tote sat min", TOTE_SAT_RANGE.minValue);
		// SmartDashboard.putNumber("Tote sat max", TOTE_SAT_RANGE.maxValue);
		// SmartDashboard.putNumber("Tote val min", TOTE_VAL_RANGE.minValue);
		// SmartDashboard.putNumber("Tote val max", TOTE_VAL_RANGE.maxValue);
		// SmartDashboard.putNumber("Area min %", AREA_MINIMUM);
		// Thread thread = new Thread();
		running = true;
		// thread.start();
		System.out.println("Starting thread");
	}

	// A structure to hold measurements of a particle
	public class ParticleReport implements Comparator<ParticleReport>,
			Comparable<ParticleReport> {
		double PercentAreaToImageArea;
		double Area;
		double ConvexHullArea;
		double BoundingRectLeft;
		double BoundingRectTop;
		double BoundingRectRight;
		double BoundingRectBottom;

		public int compareTo(ParticleReport r) {
			return (int) (r.Area - this.Area);
		}

		public int compare(ParticleReport r1, ParticleReport r2) {
			return (int) (r1.Area - r2.Area);
		}
	};

	// Structure to represent the scores for the various tests used for target
	// identification
	/*
	 * public class Scores { double Trapezoid; double LongAspect; double
	 * ShortAspect; double AreaToConvexHullArea; };
	 */
	// Images
	Image frame;
	Image binaryFrame;
	int imaqError;

	// Constants

	// // code for match
	NIVision.Range TOTE_HUE_RANGE = new NIVision.Range(24, 49); // Default hue
	// range for
	// yellow tote
	NIVision.Range TOTE_SAT_RANGE = new NIVision.Range(67, 255); // Default
	// saturation
	// range for
	// yellow
	// tote
	NIVision.Range TOTE_VAL_RANGE = new NIVision.Range(49, 255); // Default
	// value
	// range for
	// yellow
	// tote

	// ///code for auditorium
	// NIVision.Range TOTE_HUE_RANGE = new NIVision.Range(10, 50); // Default
	// hue
	// range for

	// yellow tote
	// NIVision.Range TOTE_SAT_RANGE = new NIVision.Range(90, 255); // Default
	// saturation
	// range for
	// yellow
	// tote
	// NIVision.Range TOTE_VAL_RANGE = new NIVision.Range(90, 255); // Default
	// value
	// range for
	// yellow
	// tote

	// //code for library 5 pm
	// NIVision.Range TOTE_HUE_RANGE = new NIVision.Range(25, 50); // Default
	// hue
	// range for
	// yellow tote
	// NIVision.Range TOTE_SAT_RANGE = new NIVision.Range(125, 255); // Default
	// saturation
	// range for
	// yellow
	// tote
	// NIVision.Range TOTE_VAL_RANGE = new NIVision.Range(125, 255); // Default
	// value
	// range for
	// yellow
	// tote
	double AREA_MINIMUM = 0.5; // Default Area minimum for particle as a
								// percentage of total image area
	double LONG_RATIO = 2.22; // Tote long side = 26.9 / Tote height = 12.1 =
								// 2.22
	double SHORT_RATIO = 1.4; // Tote short side = 16.9 / Tote height = 12.1 =
								// 1.4
	double SCORE_MIN = 75.0; // Minimum score to be considered a tote
	double VIEW_ANGLE = 49.4; // View angle fo camera, set to Axis m1011 by
								// default, 64 for m1013, 51.7 for 206, 52 for
								// HD3000 square, 60 for HD3000 640x480
	NIVision.ParticleFilterCriteria2 criteria[] = new NIVision.ParticleFilterCriteria2[1];
	NIVision.ParticleFilterOptions2 filterOptions = new NIVision.ParticleFilterOptions2(
			0, 0, 1, 1);

	/*
	 * Scores scores = new Scores();
	 * 
	 * // Comparator function for sorting particles. Returns true if particle 1
	 * is // larger static boolean CompareParticleSizes(ParticleReport
	 * particle1, ParticleReport particle2) { // we want descending sort order
	 * return particle1.PercentAreaToImageArea >
	 * particle2.PercentAreaToImageArea; }
	 * 
	 * /** Converts a ratio with ideal value of 1 to a score. The resulting
	 * function is piecewise linear going from (0,0) to (1,100) to (2,0) and is
	 * 0 for all inputs outside the range 0-2 183
	 */
	/*
	 * double ratioToScore(double ratio) { return (Math.max(0, Math.min(100 * (1
	 * - Math.abs(1 - ratio)), 100))); }
	 * 
	 * /** 190 * Method to score convex hull area. This scores how "complete"
	 * the particle is. Particles with large holes will score worse than a
	 * filled in shape 191
	 */
	/*
	 * double ConvexHullAreaScore(ParticleReport report) { return
	 * ratioToScore((report.Area / report.ConvexHullArea) * 1.18); }
	 * 
	 * /** 198 * Method to score if the particle appears to be a trapezoid.
	 * Compares the convex hull (filled in) area to the area of the bounding
	 * box. 199 * The expectation is that the convex hull area is about 95.4% of
	 * the bounding box area for an ideal tote. 200
	 */
	/*
	 * double TrapezoidScore(ParticleReport report) { return
	 * ratioToScore(report.ConvexHullArea / ((report.BoundingRectRight -
	 * report.BoundingRectLeft) (report.BoundingRectBottom -
	 * report.BoundingRectTop) * .954)); }
	 * 
	 * /** 207 * Method to score if the aspect ratio of the particle appears to
	 * match the long side of a tote. 208
	 */
	/*
	 * double LongSideScore(ParticleReport report) { return
	 * ratioToScore(((report.BoundingRectRight - report.BoundingRectLeft) /
	 * (report.BoundingRectBottom - report.BoundingRectTop)) / LONG_RATIO); }
	 * 
	 * /** 215 * Method to score if the aspect ratio of the particle appears to
	 * match the short side of a tote. 216
	 */
	/*
	 * double ShortSideScore(ParticleReport report) { return
	 * ratioToScore(((report.BoundingRectRight - report.BoundingRectLeft) /
	 * (report.BoundingRectBottom - report.BoundingRectTop)) / SHORT_RATIO); }
	 * 
	 * /** 222 * Computes the estimated distance to a target using the width of
	 * the particle in the image. For more information and graphics 223 *
	 * showing the math behind this approach see the Vision Processing section
	 * of the ScreenStepsLive documentation. 224 * 225 * @param image The image
	 * to use for measuring the particle estimated rectangle 226 * @param report
	 * The Particle Analysis Report for the particle 227 * @param isLong Boolean
	 * indicating if the target is believed to be the long side of a tote 228 *
	 * 
	 * @return The estimated distance to the target in feet. 229
	 */
	/*
	 * double computeDistance(Image image, ParticleReport report, boolean
	 * isLong) { double normalizedWidth, targetWidth;
	 * NIVision.GetImageSizeResult size;
	 * 
	 * size = NIVision.imaqGetImageSize(image); normalizedWidth = 2
	 * (report.BoundingRectRight - report.BoundingRectLeft) / size.width;
	 * targetWidth = isLong ? 26.0 : 16.9;
	 * 
	 * return targetWidth / (normalizedWidth * 12 * Math.tan(VIEW_ANGLE *
	 * Math.PI / (180 * 2))); }
	 */
	public void run() {
		// public void run(RGBValue colorTable) {
		double runTime;
		
		while (running) {
			runTime = Timer.getFPGATimestamp();
			System.out.println("Timer starting " + runTime);
			System.out.println("Vision is running...");
			// read file in from disk. For this example to run you need to copy
			// image20.jpg from the SampleImages folder to the
			// directory shown below using FTP or SFTP:
			// http://wpilib.screenstepslive.com/s/4485/m/24166/l/282299-roborio-ftp
			// NIVision.imaqReadFile(frame,
			// "/home/lvuser/SampleImages/image20.jpg");

			NIVision.IMAQdxStartAcquisition(session);
			
			//camera.setWhiteBalanceManual(3500);
			//camera.startCapture();
			//camera.getImage(frame);
			//camera.stopCapture();
			NIVision.IMAQdxGrab(session, frame, 1);
			// NIVision.imaqDrawShapeOnImage(frame, frame, rect,
			// DrawMode.DRAW_VALUE, ShapeMode.SHAPE_OVAL, 0.0f);

			 CameraServer.getInstance().setImage(frame);

			/** robot code here! **/
			// Timer.delay(0.005); // wait for a motor update time

			NIVision.IMAQdxStopAcquisition(session);

			// Update threshold values from SmartDashboard. For performance
			// reasons
			// it is recommended to remove this after calibration is finished.
			// TOTE_HUE_RANGE.minValue = (int) SmartDashboard.getNumber(
			// "Tote hue min", TOTE_HUE_RANGE.minValue);
			// TOTE_HUE_RANGE.maxValue = (int) SmartDashboard.getNumber(
			// "Tote hue max", TOTE_HUE_RANGE.maxValue);
			// TOTE_SAT_RANGE.minValue = (int) SmartDashboard.getNumber(
			// "Tote sat min", TOTE_SAT_RANGE.minValue);
			// TOTE_SAT_RANGE.maxValue = (int) SmartDashboard.getNumber(
			// "Tote sat max", TOTE_SAT_RANGE.maxValue);
			// TOTE_VAL_RANGE.minValue = (int) SmartDashboard.getNumber(
			// "Tote val min", TOTE_VAL_RANGE.minValue);
			// TOTE_VAL_RANGE.maxValue = (int) SmartDashboard.getNumber(
			// "Tote val max", TOTE_VAL_RANGE.maxValue);

			// Threshold the image looking for yellow (tote color)
			NIVision.imaqColorThreshold(binaryFrame, frame, 255,
					NIVision.ColorMode.HSL, TOTE_HUE_RANGE, TOTE_SAT_RANGE,
					TOTE_VAL_RANGE);

			// Send particle count to dashboard
			int numParticles = NIVision.imaqCountParticles(binaryFrame, 1);
			System.out.println("particle count =" + numParticles);
			// SmartDashboard.putNumber("Masked particles", numParticles);

			// Send masked image to dashboard to assist in tweaking mask.
			//CameraServer.getInstance().setImage(frame);
			CameraServer.getInstance().setImage(binaryFrame);

			// filter out small particles
			float areaMin = (float) SmartDashboard.getNumber("Area min %",
					AREA_MINIMUM);
			System.out.println("areaMin =" + areaMin);
			criteria[0].lower = areaMin;
			// imaqError = NIVision.imaqParticleFilter4(binaryFrame,
			// binaryFrame,
			// criteria, filterOptions, null);

			// Send particle count after filtering to dashboard
			numParticles = NIVision.imaqCountParticles(binaryFrame, 1);
			System.out.println("particle count after filtering ="
					+ numParticles);
			// SmartDashboard.putNumber("Filtered particles", numParticles);

			double biggest = 0;
			int tempindex = 0;
			if (numParticles > 0) {
				// Measure particles and sort by particle size
				Vector<ParticleReport> particles = new Vector<ParticleReport>();
				// biggest = 0;
				// tempindex = 0;
				for (int particleIndex = 0; particleIndex < numParticles; particleIndex++) {
					ParticleReport par = new ParticleReport();
					// par.PercentAreaToImageArea =
					// NIVision.imaqMeasureParticle(
					// binaryFrame, particleIndex, 0,
					// NIVision.MeasurementType.MT_AREA_BY_IMAGE_AREA);
					par.Area = NIVision.imaqMeasureParticle(binaryFrame,
							particleIndex, 0, NIVision.MeasurementType.MT_AREA);
					// par.ConvexHullArea = NIVision.imaqMeasureParticle(
					// binaryFrame, particleIndex, 0,
					// NIVision.MeasurementType.MT_CONVEX_HULL_AREA);
					// par.BoundingRectTop = NIVision.imaqMeasureParticle(
					// binaryFrame, particleIndex, 0,
					// NIVision.MeasurementType.MT_BOUNDING_RECT_TOP);
					par.BoundingRectLeft = NIVision.imaqMeasureParticle(
							binaryFrame, particleIndex, 0,
							NIVision.MeasurementType.MT_BOUNDING_RECT_LEFT);
					// par.BoundingRectBottom = NIVision.imaqMeasureParticle(
					// binaryFrame, particleIndex, 0,
					// NIVision.MeasurementType.MT_BOUNDING_RECT_BOTTOM);
					par.BoundingRectRight = NIVision.imaqMeasureParticle(
							binaryFrame, particleIndex, 0,
							NIVision.MeasurementType.MT_BOUNDING_RECT_RIGHT);
					particles.add(par);
					if (par.Area > biggest) {
						biggest = par.Area;
						tempindex = particleIndex;
						if (biggest > 2000) {
							System.out.println("Tote Found!!");
							m_foundTote = true;
						} else {
							System.out.println("Tote NOT Found!!");
							m_foundTote = false;
						}
					}
				}
				// SmartDashboard.putNumber("Index of tote", tempindex);
				horizontal = (particles.get(tempindex).BoundingRectLeft + particles
						.get(tempindex).BoundingRectRight) / 2;
				System.out.println("x coord1 =" + horizontal);
				// vertical = (particles.get(tempindex).BoundingRectTop +
				// particles
				// .get(tempindex).BoundingRectBottom) / 2;
				SmartDashboard.putNumber("x coord1 = ", horizontal);
				// SmartDashboard.putNumber("y coord", vertical);
				// SmartDashboard.putNumber("area",
				// particles.get(tempindex).Area);
				SmartDashboard.putBoolean("m_FoundTote=", m_foundTote);

				particles.sort(null);
			}

			// This example only scores the largest particle. Extending to score
			// all particles and choosing the desired one is left as an exercise
			// for the reader. Note that the long and short side scores expect a
			// single tote and will not work for a stack of 2 or more totes.
			// Modification of the code to accommodate 2 or more stacked totes
			// is left as an exercise for the reader.

			// scores.Trapezoid = TrapezoidScore(particles.elementAt(0));
			// SmartDashboard.putNumber("Trapezoid", scores.Trapezoid);
			// scores.LongAspect = LongSideScore(particles.elementAt(0));
			// SmartDashboard.putNumber("Long Aspect", scores.LongAspect);
			// scores.ShortAspect = ShortSideScore(particles.elementAt(0));
			// SmartDashboard.putNumber("Short Aspect", scores.ShortAspect);
			// scores.AreaToConvexHullArea = ConvexHullAreaScore(particles
			// .elementAt(0));
			// SmartDashboard.putNumber("Convex Hull Area",
			// scores.AreaToConvexHullArea);
			// boolean isTote = scores.Trapezoid > SCORE_MIN
			// && (scores.LongAspect > SCORE_MIN || scores.ShortAspect >
			// SCORE_MIN)
			// && scores.AreaToConvexHullArea > SCORE_MIN;
			// boolean isLong = scores.LongAspect > scores.ShortAspect;

			// Send distance and tote status to dashboard. The bounding rect,
			// particularly the horizontal center (left - right) may be useful
			// for rotating/driving towards a tote
			// SmartDashboard.putBoolean("IsTote", isTote);
			// SmartDashboard.putNumber("Distance",computeDistance(binaryFrame,particles.elementAt(0),
			// isLong));
			// } else {
			// SmartDashboard.putBoolean("IsTote", false);
			// }
			//
			// Timer.delay(0.1); // wait for a motor update time
			// we're done, so let's give others a chance to run
			System.out.println("Timer stop "
					+ (Timer.getFPGATimestamp() - runTime));
			try {
				// Thread.
				Thread.sleep((long) 0);// - (Timer.getFPGATimestamp() -
										// start_time))); //sleep just long
										// enough. =period - execution time
			} catch (Exception ex) {
				ex.printStackTrace();
			}
		}

	}

	public double getTotePosition() {
		System.out.println("In getTotePosition...");
		System.out.println("horizontal = " + horizontal);
		return horizontal;
	}

	public boolean hasFoundTote() {
		System.out.println("In hasFoundTote...");
		System.out.println("m_foundTote = " + m_foundTote);
		return m_foundTote;
	}

}
