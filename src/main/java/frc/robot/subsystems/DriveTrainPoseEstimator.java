// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.logging.Logger;

public class DriveTrainPoseEstimator extends SubsystemBase {

  private final Logger logger;
  private Swerve swerve;
  private Vision vision;
  private SwerveDrivePoseEstimator poseEstimator;
  private final org.littletonrobotics.junction.Logger aLogger;
  public DriveTrainPoseEstimator() {
    logger = Logger.getLogger(DriveTrainPoseEstimator.class.getName());
    aLogger = org.littletonrobotics.junction.Logger.getInstance();
    poseEstimator =
      new SwerveDrivePoseEstimator(
        Constants.RobotDimensions.SWERVE_DRIVE_KINEMATICS,
        swerve.getRotation2d(),
        swerve.getModulePositions(),
        new Pose2d(),
        new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.1, 0.1, 0.1),
        new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.1, 0.1, 0.1)
      );
  }

  public void addVisionMeasurement(
    Pose2d visionMeasurement,
    double timestampSeconds
  ) {
    poseEstimator.addVisionMeasurement(
      visionMeasurement,
      timestampSeconds,
      VecBuilder.fill(0, 0, 0) //TODO: Add vision measurement covariance
    );
  }

  public Pose2d getEstimatedPose() {
    return poseEstimator.getEstimatedPosition();
  }

  @Override
  public void periodic() {
    poseEstimator.updateWithTime(
      Timer.getFPGATimestamp(),
      swerve.getRotation2d(),
      swerve.getModulePositions()
    );
    aLogger.recordOutput("PoseEstimator/position", getEstimatedPose());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
