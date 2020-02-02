/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {

	private final WPI_TalonFX leftMain;
	private final WPI_TalonFX leftSlave;
	private final WPI_TalonFX rightMain;
	private final WPI_TalonFX rightSlave;
	private final DoubleSolenoid shifter;

	private final Gyro m_gyro;

	private final DifferentialDrive m_drive;

	private final DifferentialDriveOdometry m_odometry;

	public DriveSubsystem() {
		leftMain = new WPI_TalonFX(Constants.leftMainPort);
		leftSlave = new WPI_TalonFX(Constants.leftSlavePort);
		rightMain = new WPI_TalonFX(Constants.rightMainPort);
		rightSlave = new WPI_TalonFX(Constants.rightSlavePort);
		shifter = new DoubleSolenoid(Constants.shifterChannels[0], Constants.shifterChannels[1]);
		m_gyro = new ADXRS450_Gyro();
		m_drive = new DifferentialDrive(leftMain, rightMain);
		m_odometry = new DifferentialDriveOdometry(new Rotation2d());

		leftSlave.follow(leftMain);
		rightSlave.follow(rightMain);
		leftMain.configVoltageCompSaturation(Constants.drivetrainMaxVoltage);
		rightMain.configVoltageCompSaturation(Constants.drivetrainMaxVoltage);
		leftMain.configOpenloopRamp(Constants.drivetrainRampRate);
		rightMain.configOpenloopRamp(Constants.drivetrainRampRate);
		leftMain.enableVoltageCompensation(true);
		rightMain.enableVoltageCompensation(true);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		Rotation2d gyroAngle = Rotation2d.fromDegrees(-m_gyro.getAngle());
		m_odometry.update(gyroAngle, leftMain.getSensorCollection().getIntegratedSensorPosition(),
				rightMain.getSensorCollection().getIntegratedSensorPosition());
	}

	public void upshift() {
		shifter.set(Value.kReverse);
	}

	public void downshift() {
		shifter.set(Value.kForward);
	}

	public void manualDrive(double speed, double turn) {
		m_drive.arcadeDrive(speed, turn, true);
	}

	public void tankDriveVolts(double left, double right) {
		leftMain.setVoltage(left);
		rightMain.setVoltage(right);
	}

	public Pose2d getPose() {
		return m_odometry.getPoseMeters();
	}

	public DifferentialDriveWheelSpeeds getWheelSpeeds() {
		return new DifferentialDriveWheelSpeeds(leftMain.getSelectedSensorVelocity(),
				rightMain.getSelectedSensorVelocity());
	}

}
