package frc.team1706.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Victor;

public class Hand {
	static Victor left = new Victor(0);
	static Victor right = new Victor(1);

	static DoubleSolenoid grabber = new DoubleSolenoid(0, 1);

	static double motorCommand;

	public static void set(String state) {
		switch (state) {
			case "Open":
				grabber.set(DoubleSolenoid.Value.kForward);
				motorCommand = 0.0;

				break;

			case "Pull":
				grabber.set(DoubleSolenoid.Value.kReverse);
				motorCommand = 1.0;

				break;

			case "Hold":
				grabber.set(DoubleSolenoid.Value.kReverse);
				motorCommand = 0.0;

				break;

			case "Push":
				grabber.set(DoubleSolenoid.Value.kReverse);
				motorCommand = -1.0;

				break;
		}

		left.set(motorCommand);
		right.set(-motorCommand);

	}
}
