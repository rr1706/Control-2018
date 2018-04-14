package frc.team1706.robot.subsystems;

import edu.wpi.first.wpilibj.PWMSpeedController;

public class Ziptie {
	private static PWMSpeedController tie1 = new PWMSpeedController(15) {};
	private static PWMSpeedController tie2 = new PWMSpeedController(16) {};


	public static void deploy() {
		tie1.set(0.6);
		tie2.set(-0.6);
	}

	public static void stop() {
		tie1.set(-0.3);
		tie2.set(0.3);
	}
}
