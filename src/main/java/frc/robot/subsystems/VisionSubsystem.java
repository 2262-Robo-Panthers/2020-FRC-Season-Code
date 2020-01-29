/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {

	private final NetworkTable m_table;
	private final NetworkTableEntry poseEntry;
	private final NetworkTableEntry latencyEntry;

	/**
	 * Creates a new VisionSubsystem.
	 */
	public VisionSubsystem() {
		m_table = NetworkTableInstance.getDefault().getTable("chameleon-vision/Microsoft_LifeCam_HD_3000");
		poseEntry = m_table.getEntry("targetPose");
		latencyEntry = m_table.getEntry("latency");
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}

	public Pose2d getTargetPose() {
		double[] poseArray = poseEntry.getDoubleArray((double[])null);
		return new Pose2d(poseArray[0], poseArray[1], Rotation2d.fromDegrees(poseArray[2]));
	}

	public double getLatency() {
		return latencyEntry.getDouble(0.0);
	}

}
