/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

	private final DoubleSupplier hoodSupplier;

	private final CANSparkMax m_wheelMotor;

	private final WPI_VictorSPX m_hoodMotor;
	private final Encoder m_hoodEncoder;

	private final Spark m_topConveyorMotor;
	private final Spark m_bottomConveyorMotor;

	private final DoubleSolenoid m_shooterGate;

	private final DigitalInput m_topSensor;
	private final DigitalInput m_bottomSensor;

	/**
	 * Creates a new ShooterSubsystem.
	 */
	public ShooterSubsystem(DoubleSupplier hoodSupplier) {

		this.hoodSupplier = hoodSupplier;

		m_wheelMotor = new CANSparkMax(Constants.flywheelMotorPort, MotorType.kBrushless);
		m_hoodMotor = new WPI_VictorSPX(Constants.shooterHoodPort);
		m_hoodEncoder = new Encoder(Constants.shooterHoodEncoderChannels[0], Constants.shooterHoodEncoderChannels[1]);
		m_topConveyorMotor = new Spark(Constants.topConveyorMotorPort);
		m_bottomConveyorMotor = new Spark(Constants.bottomConveyorMotorPort);
		m_shooterGate = new DoubleSolenoid(Constants.PCMPort, Constants.shooterGateChannels[0], Constants.shooterGateChannels[1]);
		m_topSensor = new DigitalInput(Constants.topConveyorSensorPort);
		m_bottomSensor = new DigitalInput(Constants.bottomConveyorSensorPort);

		m_wheelMotor.setInverted(false);
		m_wheelMotor.setIdleMode(IdleMode.kCoast);
		m_wheelMotor.set(0.0);
		setGateClosed(true);
	}

	public double getHoodDistance() {
		return m_hoodEncoder.getDistance();
	}

	public void setHoodVolts(double volts) {
		m_hoodMotor.setVoltage(-volts);
	}

	public double getWheelVelocity() {
		return -m_wheelMotor.getEncoder().getVelocity() / 60.0;
	}

	public void setWheelVolts(double voltage) {
		m_wheelMotor.setVoltage(-voltage);
	}

	public void runConveyor() {
		m_topConveyorMotor.setSpeed(-1.0);
		m_bottomConveyorMotor.setSpeed(1.0);
	}

	public void setGateClosed(boolean setpoint) {
		m_shooterGate.set(setpoint ? Value.kForward : Value.kReverse);
	}

	@Override
	public void periodic() {
		m_hoodMotor.set(hoodSupplier.getAsDouble());
		if (m_topSensor.get()) runConveyor();
	}

	public boolean getTopSensor() {
		return m_topSensor.get();
	}

	public boolean getBottomSensor() {
		return m_bottomSensor.get();
	}

}
