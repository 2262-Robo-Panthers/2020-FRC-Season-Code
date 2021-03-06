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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.TurnToGoalCommand;
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
	public final Compressor m_compressor;

	private final XboxController m_controller;
	private final JoystickButton leftBumper;
	private final JoystickButton rightBumper;
	private final JoystickButton buttonA;
	private final JoystickButton buttonB;
	private final JoystickButton buttonX;
	private final JoystickButton buttonY;
	private final JoystickButton startButton;
	private final JoystickButton backButton;
	private final Trigger dPadUp;
	private final Trigger dPadRight;
	private final Trigger dPadDown;
	private final Trigger dPadLeft;

	private final DriveSubsystem m_driveSubsystem;
	private final IntakeSubsystem m_intakeSubsystem;
	private final ShooterSubsystem m_shooterSubsystem;
	private final LiftSubsystem m_liftSubsystem;
	private final VisionSubsystem m_visionSubsystem;

	private final ShootCommand shoot;
	private final TurnToGoalCommand turnToGoal;
	private final PIDCommand flywheelPID;
	private final RunCommand flywheelFast;

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {

		m_controller = new XboxController(Constants.controllerPort);
		leftBumper = new JoystickButton(m_controller, Button.kBumperLeft.value);
		rightBumper = new JoystickButton(m_controller, Button.kBumperRight.value);
		buttonA = new JoystickButton(m_controller, Button.kA.value);
		buttonB = new JoystickButton(m_controller, Button.kB.value);
		buttonX = new JoystickButton(m_controller, Button.kX.value);
		buttonY = new JoystickButton(m_controller, Button.kY.value);
		startButton = new JoystickButton(m_controller, Button.kStart.value);
		backButton = new JoystickButton(m_controller, Button.kBack.value);
		dPadUp = new Trigger(() -> m_controller.getPOV() == 0);
		dPadRight = new Trigger(() -> m_controller.getPOV() == 90);
		dPadDown = new Trigger(() -> m_controller.getPOV() == 180);
		dPadLeft = new Trigger(() -> m_controller.getPOV() == 270);
		m_driveSubsystem = new DriveSubsystem(
			() -> m_controller.getY(Hand.kLeft),
			() -> -m_controller.getX(Hand.kLeft)
		);
		m_intakeSubsystem = new IntakeSubsystem();
		m_shooterSubsystem = new ShooterSubsystem(() -> m_controller.getY(Hand.kRight));
		m_liftSubsystem = new LiftSubsystem(
			() -> m_controller.getTriggerAxis(Hand.kLeft),
			() -> m_controller.getTriggerAxis(Hand.kRight)
		);
		m_visionSubsystem = new VisionSubsystem();
		m_compressor = new Compressor(Constants.PCMPort);
		shoot = new ShootCommand(m_shooterSubsystem);
		turnToGoal = new TurnToGoalCommand(m_driveSubsystem, m_visionSubsystem);
		flywheelPID = new PIDCommand(
			m_shooterSubsystem.getController(),
			m_shooterSubsystem::getWheelVelocity,
			m_shooterSubsystem::getSetpoint,
			output -> m_shooterSubsystem.setWheelVolts(output + Constants.flywheelFF.calculate(m_shooterSubsystem.getSetpoint()))
		);
		flywheelFast = new RunCommand(() -> m_shooterSubsystem.setWheelVolts(12));
		// Configure the button bindings
		configureButtonBindings();

	}

	/**
	 * Use this method to define your button->command mappings. Buttons can be
	 * created by instantiating a {@link GenericHID} or one of its subclasses
	 * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
	 * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
	 */
	private void configureButtonBindings() {
		leftBumper.whenPressed(m_driveSubsystem::downshift);
		rightBumper.whenPressed(m_driveSubsystem::upshift);
		dPadLeft.whenActive(flywheelPID);
		dPadRight.whenActive(flywheelFast);
		dPadDown.whenActive(() -> m_intakeSubsystem.runDeployMotor(false))
			.whenInactive(m_intakeSubsystem::stopDeployMotor);
		dPadUp.whenActive(() -> m_intakeSubsystem.runDeployMotor(true))
			.whenInactive(m_intakeSubsystem::stopDeployMotor);
		buttonA.whenPressed(shoot);
		buttonB.whenPressed(
			() -> {
				m_shooterSubsystem.setWheelVolts(0.0);
				if (flywheelPID.isScheduled()) flywheelPID.end(false);
				if (flywheelFast.isScheduled()) flywheelFast.end(false);
			}
		);
		buttonX.toggleWhenPressed(new RunCommand(m_intakeSubsystem::spinRoller));
		buttonY.whenPressed(turnToGoal);
		startButton.whenPressed(() -> m_liftSubsystem.setPistonExtended(true));
		backButton.whenPressed(() -> m_liftSubsystem.setPistonExtended(false));
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		return null;
	}
}
