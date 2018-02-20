package frc.team1706.robot.subsystems;

import edu.wpi.first.wpilibj.VictorSP;

public class Winch {
	private static VictorSP motor1 = new VictorSP(10);
	private static VictorSP motor2 = new VictorSP(11);

	public static void set(double speed) {
		motor1.set(speed);
		motor2.set(speed);
	}
}
