/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class TurnToGoalCommand extends PIDCommand {

	DriveSubsystem m_driveSubsystem;
	VisionSubsystem m_visonSubsystem;

	public final double setpoint;
	public final double originalAngle;

	public TurnToGoalCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {

		super(new PIDController(1, 0, 0), () -> 0, () -> 0, output -> {});

		m_driveSubsystem = driveSubsystem;
		m_visonSubsystem = visionSubsystem;

		setpoint = m_visonSubsystem.getTargetPose().getRotation().getDegrees();
		originalAngle = m_driveSubsystem.getAngle();

		m_measurement = this::getMeasurement;
		m_setpoint = () -> setpoint;
		m_useOutput = output -> driveSubsystem.tankDriveVolts(output, -output);

	}

	private double getMeasurement() {
		return m_driveSubsystem.getAngle() - originalAngle;
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return getController().getPositionError() < 2 && getController().getPositionError() > -2;
	}

}
