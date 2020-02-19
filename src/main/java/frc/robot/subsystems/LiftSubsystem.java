/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LiftSubsystem extends SubsystemBase {

	private final DoubleSupplier leftTrigger;
	private final DoubleSupplier rightTrigger;

	private final CANSparkMax m_motor;
	private final Solenoid m_piston;

	/**
	 * Creates a new LiftSubsystem.
	 */
	public LiftSubsystem(DoubleSupplier leftTrigger, DoubleSupplier rightTrigger) {
		this.leftTrigger = leftTrigger;
		this.rightTrigger = rightTrigger;

		m_motor = new CANSparkMax(Constants.liftMotorPort, MotorType.kBrushless);
		m_piston = new Solenoid(Constants.PCMPort, Constants.liftSolenoidChannel);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		m_motor.set(rightTrigger.getAsDouble() - leftTrigger.getAsDouble());
	}

	public void setPistonExtended(boolean setpoint) {
		m_piston.set(setpoint);
	}

}
