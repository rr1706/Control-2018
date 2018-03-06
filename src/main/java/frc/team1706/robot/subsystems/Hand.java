package frc.team1706.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Victor;

public class Hand {
	private static Victor left = new Victor(6);
	private static Victor right = new Victor(7);

	private static DoubleSolenoid grabber = new DoubleSolenoid(0, 3);

	private static double motorCommand;

	public static void set(String state) {
		switch (state) {
			case "Open":
				grabber.set(DoubleSolenoid.Value.kForward);
				motorCommand = 0.0;

				break;

			case "Pull":
				grabber.set(DoubleSolenoid.Value.kReverse);
				motorCommand = 0.6;

				break;

			case "OpenPull":
				grabber.set(DoubleSolenoid.Value.kForward);
				motorCommand = 1.0;

				break;

			case "Hold":
				grabber.set(DoubleSolenoid.Value.kReverse);
				motorCommand = 0.0;

				break;

			case "Push":
				grabber.set(DoubleSolenoid.Value.kReverse);
				motorCommand = -0.6;

				break;

			case "Turbo":
				grabber.set(DoubleSolenoid.Value.kReverse);
				motorCommand = -1.0;

				break;
		}

		left.set(-motorCommand);
		right.set(motorCommand);

	}
}
