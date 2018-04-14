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

	private static int lsPort;

	private static int grabTime = 0;
	private static int grabCase = 0;

	private static final double grabPoint = -53.0;
	private static final double holdPoint = -53.0;
	private static final double switchPoint = -53.0;
	private static final double hswitchPoint = -10.0;
	private static final double lScalePoint = 8.0;
	private static final double mScalePoint = 21.0;
	private static final double hScalePoint = 47.0;
	private static final double behindPoint = 5;
	private static final double vaultPoint = 5;
	private static final double climbPoint = 70;

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

	private static int wristCheck;
	public static int armCase = 0;
	public static boolean auto = false;

	private static DigitalInput nearCubeSensor;
	private static DigitalInput farCubeSensor;
	private static DigitalInput distanceSensor;
	private static DigitalInput wallSensor;

	public static boolean haveCube = false;
	public static boolean prevHaveCube = false;
	public static boolean onCube = false;
	public static boolean nearCube = false;
	public static boolean onWall = false;

	private static boolean manual = false;
	private static int manualToggle = 0;
	private static boolean manualToggled = false;

	public static void init() {

		shoulderM = new VictorSP(4);


		wristM = new VictorSP(5);

		shoulderA = new AnalogInput(0);
		wristA = new AnalogInput(3);

		shoulderPID = new PIDController(0.06, 0.0, 0.0);
		shoulderPID.setInputRange((SHOULDER_MIN-SHOULDER_B)/SHOULDER_M, (SHOULDER_MAX-SHOULDER_B)/SHOULDER_M);
		shoulderPID.setOutputRange(-1.0, 1.0);
		shoulderPID.setContinuous(false);
		shoulderPID.setTolerance(0.2);
		shoulderPID.enable();

		wristPID = new PIDController(0.022, 0.0, 0.0);
		wristPID.setInputRange((WRIST_MIN-WRIST_B)/WRIST_M, (WRIST_MAX-WRIST_B)/WRIST_M);
		wristPID.setOutputRange(-1.0, 1.0);
		wristPID.setContinuous(false);
		wristPID.setTolerance(0.2);
		wristPID.enable();

		nearCubeSensor = new DigitalInput(0);
		farCubeSensor = new DigitalInput(8);
		distanceSensor = new DigitalInput(9);
		wallSensor = new DigitalInput(7);
	}

	public static void update() {
		SmartDashboard.putBoolean("Cube Distance Sensor", distanceSensor.get());
		SmartDashboard.putBoolean("Have Cube", !nearCubeSensor.get());
		SmartDashboard.putBoolean("On Cube", farCubeSensor.get());
		SmartDashboard.putBoolean("On Wall", wallSensor.get());

		haveCube = !nearCubeSensor.get();
		onCube = farCubeSensor.get();
		nearCube = distanceSensor.get();
		onWall = wallSensor.get();
		shoulderAngle = shoulderA.getValue();
		wristAngle = wristA.getValue();

		SmartDashboard.putNumber("Shoulder AngleDeg", (shoulderAngle-SHOULDER_B)/SHOULDER_M);
		SmartDashboard.putNumber("Wrist AngleDeg", (wristAngle-WRIST_B)/WRIST_M);

		SmartDashboard.putNumber("Shoulder AnglePot", shoulderAngle);
		SmartDashboard.putNumber("Wrist AnglePot", wristAngle);

		shoulderAngle = (shoulderAngle-SHOULDER_B)/SHOULDER_M;
		wristAngle = (wristAngle-WRIST_B)/WRIST_M;

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
//		System.out.println("nearCube: " + nearCube + " | haveCube: " + haveCube);

		if (!manual || auto) {
			if (((armCase == 0 && !Robot.xbox2.B()) || armCase == 3 || armCase == 6 || armCase == 9 || armCase == 12 || armCase == 15 || armCase == 18 || armCase == 24) && !auto ) {
				if (Robot.xbox2.A() && Robot.xbox2.LStickX() > 0.5) {
					armCase = 24;
				} else if (Robot.xbox2.A() && Robot.xbox2.LStickY() > 0.5) {
					armCase = 6;
				} else if (Robot.xbox2.A() && Robot.xbox2.LStickY() < -0.5) {
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

				if (Robot.xbox2.X() && armCase != 0) {
					Hand.set("Push");
				} else if (Robot.xbox2.Y() && armCase != 0) {
					Hand.set("Turbo");
				} else if (Robot.xbox2.RB() && armCase != 0) {
					Hand.set("Open");
				} else if (!auto) {
					Hand.set("Hold");
				}
			}

			switch (armCase) {
				case 0:
					shoulderSet = holdPoint;

					if (Robot.xbox2.X()) {
						Hand.set("Push");
					} else if (Robot.xbox2.Y()) {
						Hand.set("Turbo");
					} else {
						Hand.set("Hold");
					}
					wristCheck = 2;

					if (((Robot.xbox2.B() || (Robot.xbox2.Start() && nearCube))) && !haveCube) {
						armCase = 1;
					}
					break;

				case 1:
					shoulderSet = grabPoint;

					if (Robot.xbox2.RB()) {
						wristCheck = 4;
					} else {
						wristCheck = 0;
					}

					if (!auto) {
						switch (grabCase) {
							case 0:
								Hand.set("Open");
								if (nearCube && Robot.xbox2.Start() && !haveCube) {
									grabCase = 1;
								} else if (Robot.xbox2.A() && !haveCube) {
									grabCase = 2;
								}
								break;

							case 1:
								Hand.set("OpenPull");
								if (onCube && grabTime > 10) {
									grabCase = 2;
									grabTime = 0;
								}
								grabTime++;
								break;

							case 2:
								Hand.set("Pull");
								if (!Robot.xbox2.A() && !Robot.xbox2.Start()) {
									grabCase = 0;
								}

								if (haveCube) {
									Robot.FWD = -0.5;
									if (grabTime > 10) {
										armCase = 0;
										grabCase = 0;
									}
									grabTime++;
								} else {
									grabTime = 0;
								}
								break;
						}
					}

					if ((!Robot.xbox2.B() && !Robot.xbox2.Start())) {
						armCase = 0;
					}

					break;

				case 3:
					shoulderSet = switchPoint;
					wristCheck = 3;

					break;

				case 6:
					if (Robot.xbox2.LB()) {
						shoulderSet = lScalePoint-5;
						wristCheck = 6;
					} else {
						shoulderSet = lScalePoint;
						wristCheck = 3;
					}

					break;

				case 9:
					if (Robot.xbox2.LB()) {
						shoulderSet = mScalePoint+7;
						wristCheck = 6;
					} else {
						shoulderSet = mScalePoint;
						wristCheck = 3;
					}

					break;

				case 12:
					if (Robot.xbox2.LB()) {
						shoulderSet = hScalePoint-5;
						wristCheck = 6;
					} else {
						shoulderSet = hScalePoint;
						wristCheck = 3;
					}

					break;

				case 15:
					shoulderSet = hScalePoint;
					wristCheck = 3;

					break;

				case 18:
					shoulderSet = vaultPoint;

					break;

				case 21:
					shoulderSet = climbPoint;
					updateWrist(7);

					break;

				case 24:
					shoulderSet = hswitchPoint;
					wristCheck = 3;
			}

			updateWrist(wristCheck);

			if (armCase != 0) {
				shoulderSet += Robot.xbox1.RTrig()*15 - Robot.xbox1.LTrig()*15;
			}

			shoulderPID.setInput(shoulderAngle);
			shoulderPID.setSetpoint(shoulderSet);
			if (shoulderPID.performPID() < 0.0) {
				shoulderM.set(shoulderPID.performPID()*0.5);
			} else {
				shoulderM.set(shoulderPID.performPID());
			}

		} else {

			if (!Robot.xbox2.A()) {
				shoulderM.set(-Robot.xbox2.LStickY());
				wristM.set(-Robot.xbox2.RStickY());
			}

			if (Robot.xbox2.X()) {
				Hand.set("Push");
			} else if (Robot.xbox2.Y()) {
				Hand.set("Turbo");
			} else if (Robot.xbox2.A()) {
				Hand.set("Pull");
			} else if (Robot.xbox2.B()) {
				Hand.set("Open");
			} else {
				Hand.set("Hold");
			}
		}
	}

	public static void updateWrist(int check) {
		switch (check) {
			case 0:
				//Down
				wristSet = 90 - shoulderAngle;
				break;
			case 1:
				//Backward
				wristSet = 360 - shoulderAngle;
				break;
			case 2:
				//Hold
				wristSet = 260;
				break;
			case 3:
				//Place
				wristSet = 190 - shoulderAngle;
				break;

			case 4:
				//Move up slightly while grabbing
				wristSet = 180;
				break;

			case 5:
				//Switch Place
				wristSet = 190 - shoulderAngle;
				break;

			case 6:
				//Push Place
				wristSet = 180 - shoulderAngle;
				break;

			case 7:
				//Climb
				wristSet = 50;
				break;
		}

		wristPID.setSetpoint(wristSet);
		wristPID.setInput(wristAngle);

		wristM.set(-wristPID.performPID());

	}

	public static void endGame(int state) {
		shoulderAngle = (shoulderAngle-SHOULDER_B)/SHOULDER_M;
		wristAngle = (wristAngle-WRIST_B)/WRIST_M;

		switch (state) {
			case 0:
				shoulderSet = climbPoint;
				updateWrist(7);
				break;

			case 1:
				shoulderSet = holdPoint;
				updateWrist(2);
				break;
		}

		shoulderPID.setInput(shoulderAngle);
		shoulderPID.setSetpoint(shoulderSet);
		if (shoulderPID.performPID() < 0.0) {
			shoulderM.set(shoulderPID.performPID()*0.5);
		} else {
			shoulderM.set(shoulderPID.performPID());
		}
	}

	public static void setOffsets(double sm, double sb, double sMin, double sMax, double wm, double wb, double wMin, double wMax, int lsP) {
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
