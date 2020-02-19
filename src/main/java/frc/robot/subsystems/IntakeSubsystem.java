/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

	private final ShuffleboardTab tab;
	private final NetworkTableEntry encoderEntry;

	private final WPI_VictorSPX rollerMotor;
	private final WPI_VictorSPX deployMotor;
	private final Encoder deployEncoder;

	/**
	 * Creates a new IntakeSubsystem.
	 */
	public IntakeSubsystem() {
		tab = Shuffleboard.getTab("Robot");
		rollerMotor = new WPI_VictorSPX(Constants.intakeRollerPort);
		deployMotor = new WPI_VictorSPX(Constants.intakeDeployMotorPort);
		deployEncoder = new Encoder(Constants.intakeDeployEncoderChannels[0], Constants.intakeDeployEncoderChannels[1]);
		encoderEntry = tab.add("Intake Encoder Counts", 0).getEntry();
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		encoderEntry.setDouble(deployEncoder.get());
	}

	public void spinRoller() {
		rollerMotor.set(-0.4);
	}

	public void runDeployMotor(boolean backward) {
		deployMotor.set(backward ? -0.3 : 0.5);
	}

	public void stopDeployMotor() {
		deployMotor.set(0);
	}

}
