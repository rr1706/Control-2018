package frc.team1706.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;

public class CustomController extends Joystick {
	private Joystick stick;

	public CustomController(int port) {
		super(port);
		stick = new Joystick(port);
	}

	public boolean LStickUp() {
		return stick.getRawButton(0);
	}

	public boolean LStickDown() {
		return stick.getRawButton(1);
	}

	public boolean LStickLeft() {
		return stick.getRawButton(2);
	}

	public boolean LStickRight() {
		return stick.getRawButton(3);
	}

	public boolean RStickUp() {
		return stick.getRawButton(4);
	}

	public boolean RStickDown() {
		return stick.getRawButton(5);
	}

	public boolean RStickLeft() {
		return stick.getRawButton(6);
	}

	public boolean RStickRight() {
		return stick.getRawButton(7);
	}

	public boolean TopLeftButton() {
		return stick.getRawButton(8);
	}

	public boolean BottomLeftButton() {
		return stick.getRawButton(9);
	}

	public boolean BottomRightButton() {
		return stick.getRawButton(10);
	}

	public boolean TopRightButton() {
		return stick.getRawButton(11);
	}
}
