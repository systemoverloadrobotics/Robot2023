// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.DriveTrainPoseEstimator;
import frc.robot.subsystems.Vision;
import java.util.logging.Logger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class AutoAdjust extends CommandBase {
  private final Logger logger;
  private final Swerve swerve;
  private final DriveTrainPoseEstimator poseEstimator;
  private final Vision vision;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutoAdjust(DriveTrainPoseEstimator poseEstimator, Swerve swerve, Vision vision) {
    logger = Logger.getLogger(AutoAdjust.class.getName());
    this.poseEstimator = poseEstimator;
    this.swerve = swerve;
    this.vision = vision;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(poseEstimator, vision, swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d currentPose = poseEstimator.getEstimatedPose();

    Transform3d target = vision.getBestTarget().getBestCameraToTarget();
    double tagDistance = target.getTranslation().getX();
    // double reportedZAngle = -Math.toDegrees(target.getRotation().getZ());
    // double zAngle = Math.copySign(180 - Math.abs(reportedZAngle), reportedZAngle);
    // double xTranslation = Math.toDegrees(Math.sin(Math.toRadians(90 - zAngle))) / (tagDistance * Math.toDegrees(Math.sin(Math.toRadians(zAngle))));
    // double slope = -tagDistance/xTranslation;



    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}