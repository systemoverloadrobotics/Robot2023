// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Swerve;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrainPoseEstimator;
import frc.robot.subsystems.Vision;

import java.util.ArrayList;
import java.util.logging.Logger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.sorutil.path.AsyncTrajectory;

/** An example command that uses an example subsystem. */
public class AutoAdjust extends CommandBase {
  private final Logger logger;
  private final Swerve swerve;
  private final DriveTrainPoseEstimator poseEstimator;
  private final Vision vision;
  private Pose2d currentPose;
  private Pose2d targetPose;
  private int grid;

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
    currentPose = poseEstimator.getEstimatedPose();
    addRequirements(poseEstimator, vision, swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(grid == 1 || grid == 3)
    switch(grid){
      case 1:
       AsyncTrajectory.generateTrajectory(currentPose, targetPose, new ArrayList<>(), Constants.Scoring.SCORING_TRAJECTORY_CONFIG);
      case 2:

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