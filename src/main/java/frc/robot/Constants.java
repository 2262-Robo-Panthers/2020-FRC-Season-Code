/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

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

	// Ports 'n Stuff - BORING
	public static final int leftMainPort = 1;
	public static final int leftSlavePort = 2;
	public static final int rightMainPort = 3;
	public static final int rightSlavePort = 4;

	public static final int[] shifterPorts = { 5, 6 };

	public static final int flywheelMotorPort = 7;
	public static final int shooterHoodPort = 8;
	public static final int[] shooterHoodEncoderPorts = { 9, 10 };
	public static final int topConveyorMotorPort = 11;
	public static final int bottomConveyorMotorPort = 12;
	public static final int[] shooterGatePorts = { 13, 14 };

	public static final int intakeRollerPort = 15;
	public static final int intakeDeployMotorPort = 16;
	public static final int[] intakeDeployEncoderPort = { 17, 18 };

	public static final int liftMotorPort = 19;
	public static final int[] liftSolenoidPorts = {20, 21};

	// Do controllers use the same ports? ¯\_(ツ)_/¯
	public static final int controllerPort = 19;
	public static final int leftBumperPort = 20;
	public static final int rightBumperPort = 21;

	// Flywheel speed in RPM
	public static final double flywheelSpeed = 2450.0;

	// Flywheel PID constants
	public static final double flywheelKP = 1.0;
	public static final double flywheelKI = 0.0;
	public static final double flywheelKD = 0.0;

	// Flywheel feedforward constants
	public static final double flywheelKS = 0.0;
	public static final double flywheelKV = 0.0;

	// So now there's PID on the shooter hood (probably)
	public static final double hoodKP = 1.0;
	public static final double hoodKI = 0.0;
	public static final double hoodKD = 0.0;

	// Voltage comp constants
	public static final double drivetrainMaxVoltage = 10.0; // This is in Volts, crazy stuff
	public static final double drivetrainRampRate = 0.5; // Seconds, crazy I know

	// Drivetrain constants for Ramsyeet
	public static final DifferentialDriveKinematics drivetrainKinematics = new DifferentialDriveKinematics(0.68);
	public static final double drivetrainKS = 0.0;
	public static final double drivetrainKV = 0.0;
	public static final double drivetrainKA = 0.0;
	public static final double drivetrainKP = 1.0;
	public static final double drivetrainKI = 0.0;
	public static final double drivetrainKD = 0.0;

	public static final double maxVelocity = 15.0;
	public static final double maxAcceleration = 7.0;

}
