/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;

public class ShooterSubsystem extends PIDSubsystem {

	private final CANSparkMax m_wheelMotor;

	private final WPI_VictorSPX m_hoodMotor;
	private final Encoder m_hoodEncoder;

	private final Spark m_topConveyorMotor;
	private final Spark m_bottomConveyorMotor;

	private final DoubleSolenoid m_shooterGate;

	/**
	 * Creates a new ShooterSubsystem.
	 */
	public ShooterSubsystem() {
		super(
				// The PIDController used by the subsystem
				new PIDController(Constants.flywheelKP, 0.0, 0.0));

		m_wheelMotor = new CANSparkMax(Constants.flywheelMotorPort, MotorType.kBrushless);
		m_hoodMotor = new WPI_VictorSPX(Constants.shooterHoodPort);
		m_hoodEncoder = new Encoder(Constants.shooterHoodEncoderChannels[0], Constants.shooterHoodEncoderChannels[1]);
		m_topConveyorMotor = new Spark(Constants.topConveyorMotorPort);
		m_bottomConveyorMotor = new Spark(Constants.bottomConveyorMotorPort);
		m_shooterGate = new DoubleSolenoid(Constants.shooterGateChannels[0], Constants.shooterGateChannels[1]);

		m_wheelMotor.setIdleMode(IdleMode.kCoast);
		setGateClosed(true);
	}

	@Override
	public void useOutput(double output, double setpoint) {
		// Use the output here
		m_wheelMotor.setVoltage(Constants.flyweelFF.calculate(setpoint) + output);
	}

	@Override
	public double getMeasurement() {
		// Return the process variable measurement here
		return m_wheelMotor.getEncoder().getVelocity();
	}

	public boolean isReady() {
		return (getMeasurement() > getController().getSetpoint() - 50
				&& getMeasurement() < getController().getSetpoint() + 50);
	}

	public double getHoodSetpoint() {
		return 0.0;
	}

	public double getHoodDistance() {
		return m_hoodEncoder.getDistance();
	}

	public void setHoodVoltage(double volts) {
		m_hoodMotor.setVoltage(volts);
	}

	public void runConveyor(double speed) {
		m_topConveyorMotor.setSpeed(speed);
		m_bottomConveyorMotor.setSpeed(-speed);
	}

	public void setGateClosed(boolean setpoint) {
		m_shooterGate.set(setpoint ? Value.kForward : Value.kReverse);
	}

	public boolean isClosed() {
		return m_shooterGate.get() == Value.kForward;
	}
}
