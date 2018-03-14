package frc.team1706.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public class Platform {
	private static DoubleSolenoid release = new DoubleSolenoid(1, 2);

	public static void release() {
		release.set(DoubleSolenoid.Value.kForward);
	}

	public static void reset() {
		release.set(DoubleSolenoid.Value.kReverse);
	}
}
