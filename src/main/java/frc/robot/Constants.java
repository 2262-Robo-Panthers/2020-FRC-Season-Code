/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

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

	public static final int flywheelMotorPort = 5;

	public static final int leftStickPort = 6;
	public static final int rightStickPort = 7;

	// Flywheel speed in RPM
	public static final double flywheelSpeed = 2920.0;

	// Flywheel PID constants
	public static final double flywheelKP = 1.0;
	public static final double flywheelKI = 0.0;
	public static final double flywheelKD = 0.0;

	// Flywheel feedforward constants
	// TODO - Use WPILib characterization to find
	public static final double flywheelKS = 0.0;
	public static final double flywheelKV = 0.0;

	// This is in Volts, crazy stuff
	public static final double drivetrainMaxVoltage = 10.0;

}
