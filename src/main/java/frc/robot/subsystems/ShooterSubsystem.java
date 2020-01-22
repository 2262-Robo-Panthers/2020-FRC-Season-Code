/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;

public class ShooterSubsystem extends PIDSubsystem {

	public CANSparkMax m_motor = new CANSparkMax(Constants.flywheelMotorPort, MotorType.kBrushless);
	public SimpleMotorFeedforward ff = new SimpleMotorFeedforward(Constants.flywheelKS,
			Constants.flywheelKV);

	/**
	 * Creates a new ShooterSubsystem.
	 */
	public ShooterSubsystem() {
		super(
				// The PIDController used by the subsystem
				new PIDController(Constants.flywheelKP, Constants.flywheelKI, Constants.flywheelKD));
	}

	@Override
	public void useOutput(double output, double setpoint) {
		// Use the output here
		m_motor.setVoltage(ff.calculate(setpoint) + output);
	}

	@Override
	public double getMeasurement() {
		// Return the process variable measurement here
		return m_motor.getEncoder().getVelocity();
	}
}
