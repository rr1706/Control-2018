package frc.team1706.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.VictorSP;
import frc.team1706.robot.Robot;
import frc.team1706.robot.utilities.MathUtils;
import frc.team1706.robot.utilities.PIDController;

public class Arm {
	private static final double UPPERARM_LENGTH = 25.0;
	private static final double FOREARM_LENGTH = 31.0;

	private static double SHOULDER_M;
	private static double SHOULDER_B;

	private static double SHOULDER_MIN;
	private static double SHOULDER_MAX;

	private static double ELBOW_M;
	private static double ELBOW_B;

	private static double ELBOW_MIN;
	private static double ELBOW_MAX;

	private static double WRIST_M;
	private static double WRIST_B;

	private static double WRIST_MIN;
	private static double WRIST_MAX;

	private static final double speed = 5; //Inches per Second

	private static final double[] grabPoint = {38.462, -30.083};
	private static final double[] holdPoint = {28.755, 12.387};
	private static final double[] switchPoint = {30.933, 12.254};
	private static final double[] lScalePoint = {19.966, 15.955};
	private static final double[] mScalePoint = {24.364, 28.816};
	private static final double[] hScalePoint = {23.554, 40.764};
	private static final double[] behindPoint = {-11.734, 44.1};
	private static final double[] vaultPoint = {30.642, -31.97};
	private static final double[] climbPoint = {};

	private static VictorSP shoulderM;
	private static VictorSP elbowM;
	private static VictorSP wristM;

	private static AnalogInput shoulderA;
	private static AnalogInput elbowA;
	private static AnalogInput wristA;

	private static double shoulderAngle = 0.0;
	private static double elbowAngle = 90.0;
	private static double wristAngle = 90.0;

	private static PIDController shoulderPID;
	private static PIDController elbowPID;
	private static PIDController wristPID;

	private static double xTravel = holdPoint[0];
	private static double yTravel = holdPoint[1];
	private static  double wristSet = 90.0;

	private static double xSetpoint = holdPoint[0];
	private static double ySetpoint = holdPoint[1];

	private static double max = 1.0;

	private static int armCase = 0;

	private static DigitalInput limitSwitch = new DigitalInput(0);
	private static boolean haveCube;

	private static boolean prevB = false;

	private static boolean manual = false;
	private static int manualToggle = 0;
	private static boolean manualToggled = false;

	public static void init() {
		shoulderM = new VictorSP(0);
		elbowM = new VictorSP(1);
		wristM = new VictorSP(2);

		shoulderA = new AnalogInput(0);
		elbowA = new AnalogInput(1);
		wristA = new AnalogInput(2);

		shoulderPID = new PIDController(0.06, 0.0, 0.0);
		shoulderPID.setInputRange(SHOULDER_MIN, SHOULDER_MAX);
		shoulderPID.setOutputRange(-1.0, 1.0);
		shoulderPID.setContinuous(false);
		shoulderPID.setTolerance(0.2);
		shoulderPID.enable();

		elbowPID = new PIDController(0.022, 0.0, 0.0);
		elbowPID.setInputRange(ELBOW_MIN, ELBOW_MAX);
		elbowPID.setOutputRange(-1.0, 1.0);
		elbowPID.setContinuous(false);
		elbowPID.setTolerance(0.2);
		elbowPID.enable();

		wristPID = new PIDController(0.022, 0.0, 0.0);
		wristPID.setInputRange(WRIST_MIN, WRIST_MAX);
		wristPID.setOutputRange(-1.0, 1.0);
		wristPID.setContinuous(false);
		wristPID.setTolerance(0.2);
		wristPID.enable();
	}

	public static void update() {
		haveCube = limitSwitch.get();
		shoulderAngle = shoulderA.getValue();
		elbowAngle = elbowA.getValue();
		wristAngle = wristA.getValue();

		switch (manualToggle) {
			case 0:
				if (Robot.xbox2.LStickButton() && Robot.xbox2.RStickButton()) {
					manualToggle = 1;
				}
				break;

			case 1:
				if (!manualToggled) {
					manual = !manual;
					manualToggled = true;
				}

				if (!Robot.xbox2.LStickButton() && !Robot.xbox2.RStickButton()) {
					manualToggle = 0;
					manualToggled = false;
				}
				break;
		}

		if (!manual) {

			if (armCase == 0 || armCase == 3 || armCase == 6 || armCase == 9 || armCase == 12 || armCase == 15 || armCase == 18) {
				if (Robot.xbox2.A() && Robot.xbox2.LStickX() > 0.5) {
					armCase = 3;
				} else if (Robot.xbox2.A() && Robot.xbox2.LStickY() < -0.5) {
					armCase = 6;
				} else if (Robot.xbox2.A() && Robot.xbox2.LStickX() > 0.5) {
					armCase = 12;
				} else if (Robot.xbox2.A() && Robot.xbox2.LStickX() < -0.5) {
					armCase = 15;
				} else if (Robot.xbox2.A() && Robot.xbox2.LStickButton()) {
					armCase = 18;
				} else if (Robot.xbox2.A()) {
					armCase = 9;
				} else {
					armCase = 0;
				}

				if (Robot.xbox2.B() && armCase != 0) {
					Hand.set("Push");
				} else if (Robot.xbox2.RB() && armCase != 0) {
					Hand.set("Open");
				} else {
					Hand.set("Hold");
				}
			}

			prevB = Robot.xbox2.B();

			switch (armCase) {
				case 0:
					xSetpoint = holdPoint[0];
					ySetpoint = holdPoint[1];

					Hand.set("Hold");
					updateWrist(2);

					if (Robot.xbox2.B()) {
						armCase = 1;
					}
					break;

				case 1:
					xSetpoint = grabPoint[0];
					ySetpoint = grabPoint[1];

					if (Robot.xbox2.A() && !haveCube) {
						Hand.set("Pull");
					} else if (!Robot.xbox2.B()) {
						armCase = 0;
					} else {
						Hand.set("Hold");
					}

					break;

				case 3:
					xSetpoint = switchPoint[0];
					ySetpoint = switchPoint[1];

					break;

				case 6:
					xSetpoint = lScalePoint[0];
					ySetpoint = lScalePoint[1];

					break;

				case 9:
					xSetpoint = mScalePoint[0];
					ySetpoint = mScalePoint[1];

					break;

				case 12:
					xSetpoint = hScalePoint[0];
					ySetpoint = hScalePoint[1];

					break;

				case 15:
					xSetpoint = behindPoint[0];
					ySetpoint = behindPoint[1];

					break;

				case 18:
					xSetpoint = vaultPoint[0];
					ySetpoint = vaultPoint[1];

					break;
			}

			if (Math.abs(xSetpoint - xTravel) > Math.abs(ySetpoint - yTravel)) {
				max = Math.abs(xSetpoint - xTravel);
			} else {
				max = Math.abs(ySetpoint - yTravel);
			}

			if (xTravel != xSetpoint) {
				xTravel += Math.signum(xSetpoint - xTravel) * speed / 50 * Math.abs(xSetpoint - xTravel) / max;
			}

			if (yTravel != ySetpoint) {
				yTravel += Math.signum(ySetpoint - yTravel) * speed / 50 * Math.abs(ySetpoint - yTravel) / max;
			}

			if (armCase == 18) {
				updateWrist(1);
			} else if (yTravel < -10.0) {
				updateWrist(0);
			} else if (xTravel < 0.0) {
				updateWrist(3);
			} else {
				updateWrist(1);
			}

			setPoint(xTravel, yTravel);
		}

		shoulderM.set(Robot.xbox2.LStickY());
		elbowM.set(Robot.xbox2.RStickY());
		if (Math.abs(Robot.xbox2.LTrig()) > 0.6) {
			wristM.set(Robot.xbox2.LTrig());
		} else if (Math.abs(Robot.xbox2.RTrig()) > 0.6) {
			wristM.set(-Robot.xbox2.RTrig());
		}

	}

	private static void updateWrist(int check) {
		switch (check) {
			case 0:
				wristSet = 90 - MathUtils.radToDeg(shoulderAngle + elbowAngle);
				break;
			case 1:
				wristSet = 360 - MathUtils.radToDeg(shoulderAngle + elbowAngle);
				break;
			case 2:
				wristSet = 90;
				break;
			case 3:
				wristSet = 180 - MathUtils.radToDeg(shoulderAngle + elbowAngle);
				break;
		}

		wristPID.setSetpoint(WRIST_M * wristSet + WRIST_B);
		wristPID.setInput(wristAngle);

		wristM.set(wristPID.performPID());
	}

	public static void setOffsets(double sm, double sb, double sMin, double sMax, double em, double eb, double eMin, double eMax, double wm, double wb, double wMin, double wMax) {
		SHOULDER_M = sm;
		SHOULDER_B = sb;
		SHOULDER_MIN = sMin;
		SHOULDER_MAX = sMax;

		ELBOW_M = em;
		ELBOW_B = eb;
		ELBOW_MIN = sMin;
		ELBOW_MAX = sMax;

		WRIST_M = wm;
		WRIST_B = wb;
		WRIST_MIN = wMin;
		WRIST_MAX = wMax;
	}

	private static void setPoint(double x, double y) {
		double shoulderCommand = 0.0;
		double elbowCommand = 0.0;

		double[] commands = armAngles(x, y, UPPERARM_LENGTH, FOREARM_LENGTH);

		shoulderCommand = commands[0];
		elbowCommand = commands[1];

		shoulderPID.setInput(shoulderAngle);
		elbowPID.setInput(elbowAngle);

		shoulderPID.setSetpoint(SHOULDER_M * shoulderCommand + SHOULDER_B);
		elbowPID.setSetpoint(ELBOW_M * elbowCommand + ELBOW_B);

		shoulderM.set(-shoulderPID.performPID());
		elbowM.set(-elbowPID.performPID());
	}

	private static double lawOfCosines(double a, double b, double c) {
		return Math.acos((a*a + b*b - c*c) / (2*a*b));
	}

	private static double armDistance(double x, double y) {
		return Math.sqrt(x*x + y*y);
	}

	private static double[] armAngles(double x, double y, double len1, double len2) {
		double dist = armDistance(x, y);
		double d1 = Math.atan2(y, x);
		double d2 = lawOfCosines(dist, len1, len2);
		double a1 = d1 + d2;
		double a2 = lawOfCosines(len1, len2, dist);

		double[] values = new double[2];
		values[0] = a1;
		values[1] = a2;

		return values;
	}
}
