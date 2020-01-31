/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
	// The robot's subsystems and commands are defined here...
	private final DriveSubsystem m_driveSubsystem;
	private final IntakeSubsystem m_intakeSubsystem;
	private final ShooterSubsystem m_shooterSubsystem;
	private final LiftSubsystem m_liftSubsystem;
	private final VisionSubsystem m_visionSubsystem;

	public final Compressor m_compressor;

	private final XboxController m_controller;

	private final JoystickButton leftBumper;
	private final JoystickButton rightBumper;
	private final JoystickButton buttonA;
	private final JoystickButton buttonB;

	private final RunCommand teleopDrive;
	private final InstantCommand upshift;
	private final InstantCommand downshift;
	private final ShootCommand shoot;
	private final PIDCommand moveHood;

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {

		m_driveSubsystem = new DriveSubsystem();
		m_intakeSubsystem = new IntakeSubsystem();
		m_shooterSubsystem = new ShooterSubsystem();
		m_liftSubsystem = new LiftSubsystem();
		m_visionSubsystem = new VisionSubsystem();
		m_compressor = new Compressor();
		m_controller = new XboxController(Constants.controllerPort);
		leftBumper = new JoystickButton(m_controller, Button.kBumperLeft.value);
		rightBumper = new JoystickButton(m_controller, Button.kBumperRight.value);
		buttonA = new JoystickButton(m_controller, Button.kA.value);
		buttonB = new JoystickButton(m_controller, Button.kB.value);
		teleopDrive = new RunCommand(
			() -> m_driveSubsystem.manualDrive(m_controller.getY(Hand.kLeft), m_controller.getX(Hand.kRight)),
			m_driveSubsystem);
		upshift = new InstantCommand(m_driveSubsystem::upshift, m_driveSubsystem);
		downshift = new InstantCommand(m_driveSubsystem::downshift, m_driveSubsystem);
		shoot = new ShootCommand(m_shooterSubsystem);
		moveHood = new PIDCommand(
			new PIDController(Constants.hoodKP, Constants.hoodKI, Constants.hoodKD),
			m_shooterSubsystem::getHoodDistance,
			m_shooterSubsystem::getHoodSetpoint,
			(output) -> {
				m_shooterSubsystem.setHoodVoltage(output);
			},
			m_shooterSubsystem);

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
		leftBumper.whenPressed(downshift);
		rightBumper.whenPressed(upshift);
		buttonA.whenPressed(shoot);
		buttonB.whenPressed(moveHood);
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
