// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Swerve;
import frc.robot.Constants;
import frc.robot.GridSelector;
import frc.robot.GridSelector.GridLocation;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DriveTrainPoseEstimator;
import frc.robot.commands.MoveToGrid;
import frc.robot.subsystems.Vision;
import frc.sorutil.ConstantButton;
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
  private final ArmSubsystem arm;
  private final Claw claw;
  private Pose2d currentPose;
  private Pose2d ScoringLocationPose;
  private final GridLocation selectedGridLocation;
  private double offsetRight;
  private double offsetLeft;
  private final int buttonPressed;
  private Future<Trajectory> futureTrajectory;
  private boolean isTrajectoryGenerated;
  private Trajectory trajectory;
  

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public MoveToScoringLocation(DriveTrainPoseEstimator poseEstimator, Swerve swerve, Vision vision, ArmSubsystem arm, Claw claw, GridLocation selectedGridLocation, int buttonPressed) {
    logger = Logger.getLogger(MoveToScoringLocation.class.getName());
    this.poseEstimator = poseEstimator;
    this.swerve = swerve;
    this.vision = vision;
    this.arm = arm;
    this.claw = claw;
    this.selectedGridLocation = selectedGridLocation;
    this.buttonPressed = buttonPressed;
    currentPose = poseEstimator.getEstimatedPose();
    addRequirements(poseEstimator, vision, swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    switch(selectedGridLocation) {
      case RIGHT:
        offsetLeft = Constants.Scoring.RIGHT_GRID_LEFT_NODE_OFFSET;
        offsetRight = Constants.Scoring.RIGHT_GRID_RIGHT_NODE_OFFSET;
      case MIDDLE:
        offsetLeft = Constants.Scoring.MIDDLE_GRID_LEFT_NODE_OFFSET;
        offsetRight = Constants.Scoring.MIDDLE_GRID_RIGHT_NODE_OFFSET;
      case LEFT:
        offsetLeft = Constants.Scoring.LEFT_GRID_LEFT_NODE_OFFSET;
        offsetRight = Constants.Scoring.LEFT_GRID_RIGHT_NODE_OFFSET;
    }
    switch(buttonPressed % 3) {
      case 0:
        ScoringLocationPose = new Pose2d(currentPose.getX() + offsetRight, currentPose.getY(), currentPose.getRotation());
      case 1:
        ScoringLocationPose = new Pose2d(currentPose.getX() + offsetLeft, currentPose.getY(), currentPose.getRotation());
    }
    futureTrajectory = AsyncTrajectory.generateTrajectory(currentPose, ScoringLocationPose, new ArrayList<>(), Constants.Scoring.SCORING_TRAJECTORY_CONFIG);
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