// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Swerve;
import frc.robot.Constants;
import frc.robot.GridSelector.GridLocation;
import frc.robot.subsystems.DriveTrainPoseEstimator;
import frc.robot.subsystems.Vision;
import frc.sorutil.path.AsyncTrajectory;
import java.util.ArrayList;
import java.util.concurrent.Future;
import java.util.logging.Logger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class MoveToScoringLocation extends CommandBase {
  private final Logger logger;
  private final Swerve swerve;
  private final DriveTrainPoseEstimator poseEstimator;
  private final Vision vision;
  private Pose2d currentPose;
  private Pose2d targetPose;
  private final GridLocation selectedGridLocation;
  private Future<Trajectory> futureTrajectory;
  private boolean isTrajectoryGenerated;
  private Trajectory trajectory;
  

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public MoveToScoringLocation(DriveTrainPoseEstimator poseEstimator, Swerve swerve, Vision vision, GridLocation selectedGridLocation) {
    logger = Logger.getLogger(MoveToScoringLocation.class.getName());
    this.poseEstimator = poseEstimator;
    this.swerve = swerve;
    this.vision = vision;
    this.selectedGridLocation = selectedGridLocation;
    currentPose = poseEstimator.getEstimatedPose();
    addRequirements(poseEstimator, vision, swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    futureTrajectory = AsyncTrajectory.generateTrajectory(currentPose, targetPose, new ArrayList<>(), Constants.Scoring.SCORING_TRAJECTORY_CONFIG);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(futureTrajectory.isDone() && !isTrajectoryGenerated) {
      try{
        trajectory = futureTrajectory.get();
        isTrajectoryGenerated = true;
      }
      catch(Exception Exception) {
        throw new RuntimeException("MoveToGrid unreachable block");
      }
    }
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