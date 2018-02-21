package frc.team1706.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team1706.robot.Robot;
import frc.team1706.robot.utilities.MathUtils;
import frc.team1706.robot.utilities.PIDController;

public class Arm {
	private static double SHOULDER_M;
	private static double SHOULDER_B;

	private static double SHOULDER_MIN;
	private static double SHOULDER_MAX;

	private static double WRIST_M;
	private static double WRIST_B;

	private static double WRIST_MIN;
	private static double WRIST_MAX;

	private static final double speed = 5.0; //Inches per Second

	private static final double grabPoint = 5;
	private static final double holdPoint = 5;
	private static final double switchPoint = 5;
	private static final double lScalePoint = 5;
	private static final double mScalePoint = 5;
	private static final double hScalePoint = 5;
	private static final double behindPoint = 5;
	private static final double vaultPoint = 5;
	private static final double climbPoint = 5;

	public static VictorSP shoulderM;
	private static VictorSP wristM;

	private static AnalogInput shoulderA;
	private static AnalogInput wristA;

	private static double shoulderAngle = 0.0;
	private static double wristAngle = 90.0;

	private static PIDController shoulderPID;
	private static PIDController wristPID;

	private static  double shoulderSet = 0.0;
	private static  double wristSet = 90.0;

	public static int armCase = 0;

	private static DigitalInput limitSwitch = new DigitalInput(0);
	private static boolean haveCube;

	private static boolean manual = true;
	private static int manualToggle = 0;
	private static boolean manualToggled = false;

	public static void init() {
		shoulderM = new VictorSP(4);
		wristM = new VictorSP(6);

		shoulderA = new AnalogInput(0);
		wristA = new AnalogInput(2);

		shoulderPID = new PIDController(0.06, 0.0, 0.0);
		shoulderPID.setInputRange(SHOULDER_MIN, SHOULDER_MAX);
		shoulderPID.setOutputRange(-1.0, 1.0);
		shoulderPID.setContinuous(false);
		shoulderPID.setTolerance(0.2);
		shoulderPID.enable();

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
		wristAngle = wristA.getValue();

		SmartDashboard.putNumber("Shoulder AngleDeg", (shoulderAngle-SHOULDER_B)/SHOULDER_M);
		SmartDashboard.putNumber("Wrist AngleDeg", (wristAngle-WRIST_B)/WRIST_M);

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
				} else if (Robot.xbox2.A() && Robot.xbox2.LStickY() > 0.5) {
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

			switch (armCase) {
				case 0:
					shoulderSet = holdPoint;

					Hand.set("Hold");
					updateWrist(2);

					if (Robot.xbox2.B()) {
						armCase = 1;
					}
					break;

				case 1:
					shoulderSet = grabPoint;

					if (Robot.xbox2.A() && !haveCube) {
						Hand.set("Pull");
					} else if (!Robot.xbox2.B() || haveCube) {
						armCase = 0;
					}

					break;

				case 3:
					shoulderSet = switchPoint;

					break;

				case 6:
					shoulderSet = lScalePoint;

					break;

				case 9:
					shoulderSet = mScalePoint;

					break;

				case 12:
					shoulderSet = hScalePoint;

					break;

				case 15:
					shoulderSet = behindPoint;

					break;

				case 18:
					shoulderSet = vaultPoint;

					break;

				case 21:
					shoulderSet = climbPoint;

					break;
			}

			if (armCase == 18) {
				updateWrist(1);
			} else if (shoulderAngle < -10.0) {
				updateWrist(0);
			} else if (shoulderAngle > 170.0) {
				updateWrist(3);
			} else {
				updateWrist(1);
			}

			shoulderPID.setSetpoint(shoulderSet);
			shoulderM.set(shoulderPID.performPID());
		}

		shoulderM.set(Robot.xbox2.LStickY());
		if (Math.abs(Robot.xbox2.LTrig()) > 0.06) {
			wristM.set(Robot.xbox2.LTrig());
		} else if (Math.abs(Robot.xbox2.RTrig()) > 0.06) {
			wristM.set(-Robot.xbox2.RTrig());
		} else {
			wristM.set(0.0);
		}

	}

	private static void updateWrist(int check) {
		switch (check) {
			case 0:
				wristSet = 90 - MathUtils.radToDeg(shoulderAngle);
				break;
			case 1:
				wristSet = 360 - MathUtils.radToDeg(shoulderAngle);
				break;
			case 2:
				wristSet = 90;
				break;
			case 3:
				wristSet = 180 - MathUtils.radToDeg(shoulderAngle);
				break;
		}

		wristPID.setSetpoint(WRIST_M * wristSet + WRIST_B);
		wristPID.setInput(wristAngle);

		wristM.set(wristPID.performPID());
	}

	public static void setOffsets(double sm, double sb, double sMin, double sMax, double wm, double wb, double wMin, double wMax) {
		SHOULDER_M = sm;
		SHOULDER_B = sb;
		SHOULDER_MIN = sMin;
		SHOULDER_MAX = sMax;

		WRIST_M = wm;
		WRIST_B = wb;
		WRIST_MIN = wMin;
		WRIST_MAX = wMax;
	}
}
