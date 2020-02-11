/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
	private final PIDCommand flywheelSlow;
	private final PIDCommand flywheelFast;

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
		dPadUp = new Trigger(() -> {return m_controller.getPOV() == 0;});
		dPadRight = new Trigger(() -> {return m_controller.getPOV() == 90;});
		dPadDown = new Trigger(() -> {return m_controller.getPOV() == 180;});
		dPadLeft = new Trigger(() -> {return m_controller.getPOV() == 270;});
		m_driveSubsystem = new DriveSubsystem(
			() -> {return m_controller.getY(Hand.kLeft);},
			() -> {return -m_controller.getX(Hand.kLeft);}
		);
		m_intakeSubsystem = new IntakeSubsystem();
		m_shooterSubsystem = new ShooterSubsystem();
		m_liftSubsystem = new LiftSubsystem(
			() -> {return m_controller.getTriggerAxis(Hand.kLeft);},
			() -> {return m_controller.getTriggerAxis(Hand.kRight);}
		);
		m_visionSubsystem = new VisionSubsystem();
		m_compressor = new Compressor(Constants.PCMPort);
		shoot = new ShootCommand(m_shooterSubsystem);
		flywheelSlow = new PIDCommand(
			new PIDController(Constants.flywheelKP, 0.0, 0.0),
			m_shooterSubsystem::getWheelVelocity,
			Constants.flywheelLowSpeed,
			(output) -> {
				m_shooterSubsystem.setWheelVolts(output + Constants.flywheelFF.calculate(Constants.flywheelLowSpeed));
			}
		);
		flywheelFast = new PIDCommand(
			new PIDController(Constants.flywheelKP, 0.0, 0.0),
			m_shooterSubsystem::getWheelVelocity,
			Constants.flywheelHighSpeed,
			(output) -> {
				m_shooterSubsystem.setWheelVolts(output + Constants.flywheelFF.calculate(Constants.flywheelHighSpeed));
			}
		);

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
		dPadLeft.whenActive(flywheelSlow);
		dPadRight.whenActive(flywheelFast);
		dPadDown.whileActiveContinuous(() -> m_intakeSubsystem.runDeployMotor(false));
		dPadUp.whileActiveContinuous(() -> m_intakeSubsystem.runDeployMotor(true));
		buttonA.whileHeld(m_shooterSubsystem::runConveyor);
		buttonB.whenPressed(
			() -> {
				m_shooterSubsystem.setWheelVolts(0.0);
				if (flywheelSlow.isScheduled()) flywheelSlow.end(false);
				if (flywheelFast.isScheduled()) flywheelFast.end(false);
			}
		);
		buttonX.toggleWhenPressed(new RunCommand(m_intakeSubsystem::spinRoller));
		startButton.whenPressed(() -> m_liftSubsystem.setPistonExtended(true));
		backButton.whenPressed(() -> m_liftSubsystem.setPistonExtended(false));
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {

		DifferentialDriveVoltageConstraint voltageConstraint = new DifferentialDriveVoltageConstraint(
			Constants.drivetrainFF,
			Constants.drivetrainKinematics,
			Constants.drivetrainMaxVoltage
		);

		TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
			Constants.maxVelocity,
			Constants.maxAcceleration
		);
		trajectoryConfig.setKinematics(Constants.drivetrainKinematics);
		trajectoryConfig.addConstraint(voltageConstraint);

		Trajectory autoTrajectory = TrajectoryGenerator.generateTrajectory(
			new Pose2d(),
			List.of(
				new Translation2d(1, 2),
				new Translation2d(-1, 4)
			),
			new Pose2d(3, 3, new Rotation2d()),
			trajectoryConfig
		);

		RamseteCommand ramsete = new RamseteCommand(
			autoTrajectory,
			m_driveSubsystem::getPose,
			new RamseteController(),
			Constants.drivetrainFF,
			Constants.drivetrainKinematics,
			m_driveSubsystem::getWheelSpeeds,
			new PIDController(Constants.drivetrainKP, 0.0, 0.0),
			new PIDController(Constants.drivetrainKP, 0.0, 0.0),
			m_driveSubsystem::tankDriveVolts,
			m_driveSubsystem
		);

		return ramsete;
	}
}
