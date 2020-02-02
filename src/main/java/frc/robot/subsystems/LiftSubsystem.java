/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LiftSubsystem extends SubsystemBase {

	private final CANSparkMax m_motor;
	private final DoubleSolenoid m_piston;

	/**
	 * Creates a new LiftSubsystem.
	 */
	public LiftSubsystem() {
		m_motor = new CANSparkMax(Constants.liftMotorPort, MotorType.kBrushless);
		m_piston = new DoubleSolenoid(Constants.liftSolenoidChannels[0], Constants.liftSolenoidChannels[1]);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
