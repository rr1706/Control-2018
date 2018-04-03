package frc.team1706.robot.subsystems;

import edu.wpi.first.wpilibj.PWMSpeedController;

public class Ziptie {
	private static PWMSpeedController tie = new PWMSpeedController(9) {};;

	public static void deploy() {
		tie.set(0.6);
	}

	public static void stop() {
		tie.set(-1.0);
	}
}
