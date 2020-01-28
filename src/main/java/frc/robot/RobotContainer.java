/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
	// The robot's subsystems and commands are defined here...
	private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
	private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();

	private XboxController m_controller;

	private final RunCommand teleopDrive = new RunCommand(
		() -> m_driveSubsystem.manualDrive(m_controller.getY(Hand.kLeft), m_controller.getX(Hand.kRight)),
		m_driveSubsystem);
	private final PIDCommand moveHood = new PIDCommand(
		new PIDController(Constants.hoodKP, Constants.hoodKI, Constants.hoodKD),
		() -> m_shooterSubsystem.getHoodDistance(),
		() -> m_shooterSubsystem.getHoodSetpoint(),
		(output) -> {
			m_shooterSubsystem.setHoodVoltage(output);
		},
		m_shooterSubsystem);
	private final InstantCommand upshift = new InstantCommand(() -> m_driveSubsystem.upshift(), m_driveSubsystem);
	private final InstantCommand downshift = new InstantCommand(() -> m_driveSubsystem.downshift(), m_driveSubsystem);

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		// Configure the button bindings
		configureButtonBindings();

		m_driveSubsystem.setDefaultCommand(teleopDrive);
	}

	/**
	 * Use this method to define your button->command mappings. Buttons can be
	 * created by instantiating a {@link GenericHID} or one of its subclasses
	 * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
	 * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
	 */
	private void configureButtonBindings() {

	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		return teleopDrive;
	}
}
