/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCommand extends CommandBase {

	private final ShooterSubsystem m_subsystem;

	/**
	 * Creates a new ShootCommand.
	 */
	public ShootCommand(ShooterSubsystem subsystem) {
		m_subsystem = subsystem;
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		m_subsystem.setGateClosed(false);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		m_subsystem.runConveyor();
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		m_subsystem.setGateClosed(true);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return m_subsystem.getTopSensor();
	}
}
