/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {

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

		this.speedSupplier = speedSupplier;
		this.turnSupplier = turnSupplier;

		leftMain = new WPI_TalonFX(Constants.leftMainPort);
		leftSlave = new WPI_TalonFX(Constants.leftSlavePort);
		rightMain = new WPI_TalonFX(Constants.rightMainPort);
		rightSlave = new WPI_TalonFX(Constants.rightSlavePort);

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

		m_gyro = new ADXRS450_Gyro();
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
