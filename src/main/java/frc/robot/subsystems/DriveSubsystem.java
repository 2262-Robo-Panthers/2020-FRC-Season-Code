/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.analog.adis16448.frc.ADIS16448_IMU;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {

	private final ShuffleboardTab tab;
	private final NetworkTableEntry brakeEntry;
	private final NetworkTableEntry voltageEntry;
	private final NetworkTableEntry rampEntry;

	private final DoubleSupplier speedSupplier;
	private final DoubleSupplier turnSupplier;

	private final WPI_TalonFX leftMain;
	private final WPI_TalonFX leftSlave;
	private final WPI_TalonFX rightMain;
	private final WPI_TalonFX rightSlave;

	private final Solenoid shifter;

	private final Gyro m_gyro;

	private final DifferentialDrive m_drive;

	private final DifferentialDriveOdometry m_odometry;

	public DriveSubsystem(DoubleSupplier speedSupplier, DoubleSupplier turnSupplier) {

		tab = Shuffleboard.getTab("Robot");

		this.speedSupplier = speedSupplier;
		this.turnSupplier = turnSupplier;

		leftMain = new WPI_TalonFX(Constants.leftMainPort);
		leftSlave = new WPI_TalonFX(Constants.leftSlavePort);
		rightMain = new WPI_TalonFX(Constants.rightMainPort);
		rightSlave = new WPI_TalonFX(Constants.rightSlavePort);

		brakeEntry = tab.add("Brake Mode", true).getEntry();
		voltageEntry = tab.add("Drivetrain Max Voltage", Constants.drivetrainMaxVoltage).getEntry();
		rampEntry = tab.add("Ramp Rate", Constants.drivetrainRampRate).getEntry();

		leftSlave.follow(leftMain);
		rightSlave.follow(rightMain);
		leftMain.setNeutralMode(NeutralMode.Brake);
		leftSlave.setNeutralMode(NeutralMode.Brake);
		rightMain.setNeutralMode(NeutralMode.Brake);
		rightSlave.setNeutralMode(NeutralMode.Brake);
		leftMain.configVoltageCompSaturation(Constants.drivetrainMaxVoltage);
		rightMain.configVoltageCompSaturation(Constants.drivetrainMaxVoltage);
		leftMain.configOpenloopRamp(Constants.drivetrainRampRate);
		rightMain.configOpenloopRamp(Constants.drivetrainRampRate);
		leftMain.enableVoltageCompensation(true);
		rightMain.enableVoltageCompensation(true);

		shifter = new Solenoid(Constants.PCMPort, Constants.shifterChannel);
		upshift();

		m_gyro = new ADIS16448_IMU();
		m_drive = new DifferentialDrive(leftMain, rightMain);

		m_odometry = new DifferentialDriveOdometry(new Rotation2d());
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		m_drive.arcadeDrive(speedSupplier.getAsDouble(), turnSupplier.getAsDouble());
		Rotation2d gyroAngle = Rotation2d.fromDegrees(-m_gyro.getAngle());
		m_odometry.update(gyroAngle, leftMain.getSensorCollection().getIntegratedSensorPosition(),
				rightMain.getSensorCollection().getIntegratedSensorPosition());

		updateFromShuffleboard();
	}

	private void updateFromShuffleboard() {
		boolean brakeMode = brakeEntry.getBoolean(true);
		leftMain.setNeutralMode(brakeMode ? NeutralMode.Brake : NeutralMode.Coast);
		leftSlave.setNeutralMode(brakeMode ? NeutralMode.Brake : NeutralMode.Coast);
		rightMain.setNeutralMode(brakeMode ? NeutralMode.Brake : NeutralMode.Coast);
		rightSlave.setNeutralMode(brakeMode ? NeutralMode.Brake : NeutralMode.Coast);
		double maxVoltage = voltageEntry.getDouble(12.0);
		leftMain.configVoltageCompSaturation(maxVoltage);
		rightMain.configVoltageCompSaturation(maxVoltage);
		double rampRate = rampEntry.getDouble(0.2);
		leftMain.configOpenloopRamp(rampRate);
		rightMain.configOpenloopRamp(rampRate);
	}

	public void upshift() {
		shifter.set(true);
	}

	public void downshift() {
		shifter.set(false);
	}

	public Pose2d getPose() {
		return m_odometry.getPoseMeters();
	}

	public DifferentialDriveWheelSpeeds getWheelSpeeds() {
		return new DifferentialDriveWheelSpeeds(leftMain.getSelectedSensorVelocity(), rightMain.getSelectedSensorVelocity());
	}

	public double getAngle() {
		return m_gyro.getAngle();
	}

	public void tankDriveVolts(double leftSpeed, double rightSpeed) {
		leftMain.setVoltage(leftSpeed);
		rightMain.setVoltage(rightSpeed);
	}

}
