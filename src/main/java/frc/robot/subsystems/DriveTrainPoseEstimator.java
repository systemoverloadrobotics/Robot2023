// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.logging.Logger;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrainPoseEstimator extends SubsystemBase {
  private final Logger logger;
  private Swerve swerve;
  private Vision vision;
  private SwerveDrivePoseEstimator poseEstimator;
 

  public DriveTrainPoseEstimator() {
   
    logger = Logger.getLogger(DriveTrainPoseEstimator.class.getName());
    poseEstimator = new SwerveDrivePoseEstimator(Constants.RobotDimensions.SWERVE_DRIVE_KINEMATICS,
        swerve.getHeading(), swerve.getModulePositions(), new Pose2d(),
        new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.1, 0.1, 0.1),
        new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.1, 0.1, 0.1));
    var visionEstimatedPose = vision.getEstimatedGlobalPose(getEstimatedPose());
    poseEstimator.addVisionMeasurement(visionEstimatedPose.getFirst(), Timer.getFPGATimestamp());
  }

  public Pose2d getEstimatedPose() {
    return poseEstimator.getEstimatedPosition();
  }

  @Override
  public void periodic() {
    poseEstimator.updateWithTime(Timer.getFPGATimestamp(), swerve.getHeading(), swerve.getModulePositions());
   
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
