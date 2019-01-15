package frc.team1706.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;

public class Ziptie {
	private static Servo tie1 = new Servo(9);
	private static Servo tie2 = new Servo(8);

	public static void deploy() {
		tie1.set(0.3);
		tie2.set(0.8);
	}

	public static void stop() {
		tie1.set(0.65);
		tie2.set(0.45);
	}
}
