/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

	// CAN devices
	public static final int rightMainPort = 0;
	public static final int rightSlavePort = 1;
	public static final int leftMainPort = 2;
	public static final int leftSlavePort = 3;

	public static final int flywheelMotorPort = 4;
	public static final int shooterHoodPort = 5;

	public static final int intakeRollerPort = 6;
	public static final int intakeDeployMotorPort = 7;

	public static final int liftMotorPort = 8;

	public static final int PCMPort = 9;

	// PWM
	public static final int topConveyorMotorPort = 0;
	public static final int bottomConveyorMotorPort = 1;

	// Pneumatics
	public static final int[] shifterChannels = { 0, 1 };
	public static final int[] shooterGateChannels = { 2, 3 };
	public static final int[] liftSolenoidChannels = { 4, 5 };

	// Encoders
	public static final int[] shooterHoodEncoderChannels = { 0, 1 };
	public static final int[] intakeDeployEncoderChannels = { 2, 3 };

	// Xbox Controller
	public static final int controllerPort = 0;

	// Flywheel
	public static final double flywheelSpeed = 1650.0;
	public static final double flywheelKP = 1.0;
	public static final SimpleMotorFeedforward flyweelFF = new SimpleMotorFeedforward(0.0, 0.0, 0.0);

	// So now there's PID on the shooter hood (probably)
	public static final double hoodKP = 1.0;
	public static final double hoodKI = 0.0;
	public static final double hoodKD = 0.0;

	// Voltage comp constants
	public static final double drivetrainMaxVoltage = 10.0; // This is in Volts, crazy stuff
	public static final double drivetrainRampRate = 0.5; // Seconds, crazy I know

}
