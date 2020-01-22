/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {

	private WPI_TalonFX leftMain = new WPI_TalonFX(Constants.leftMainPort);
	private WPI_TalonFX leftSlave = new WPI_TalonFX(Constants.leftSlavePort);
	private WPI_TalonFX rightMain = new WPI_TalonFX(Constants.rightMainPort);
	private WPI_TalonFX rightSlave = new WPI_TalonFX(Constants.rightSlavePort);

	public DifferentialDrive m_drive = new DifferentialDrive(leftMain, rightMain);

	public DriveSubsystem() {
		leftSlave.follow(leftMain);
		rightSlave.follow(rightMain);

		leftMain.configVoltageCompSaturation(Constants.drivetrainMaxVoltage);
		rightMain.configVoltageCompSaturation(Constants.drivetrainMaxVoltage);
		leftMain.enableVoltageCompensation(true);
		rightMain.enableVoltageCompensation(true);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}

}
