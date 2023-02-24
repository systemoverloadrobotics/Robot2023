// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Swerve;
import frc.robot.Constants;
import frc.robot.GridSelector;
import frc.robot.GridSelector.GridLocation;
import frc.robot.subsystems.DriveTrainPoseEstimator;
import frc.robot.subsystems.Vision;
import frc.sorutil.path.AsyncTrajectory;
import java.util.ArrayList;
import java.util.concurrent.Future;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** Moves Bot to desired Grid */
public class MoveToGrid extends CommandBase {
  private final java.util.logging.Logger logger;
  private final org.littletonrobotics.junction.Logger aLogger;
  private final Swerve swerve;
  private final DriveTrainPoseEstimator poseEstimator;
  private final Vision vision;
  private Pose2d currentPose;
  private Pose2d tagPose;
  private Pose2d nextToTagPose;
  private int selectedGridId;
  public boolean isBotAtGrid;
  private Future<Trajectory> futureTrajectory;
  private Trajectory trajectory;
  private HolonomicDriveController controller;
  private boolean isTrajectoryGenerated;

  /**
   * Creates a new MoveToGrid Command.
   */
  public MoveToGrid(DriveTrainPoseEstimator poseEstimator, Swerve swerve, Vision vision) {
    logger = java.util.logging.Logger.getLogger(MoveToGrid.class.getName());
    aLogger = org.littletonrobotics.junction.Logger.getInstance();
    this.poseEstimator = poseEstimator;
    this.swerve = swerve;
    this.vision = vision;
    controller = new HolonomicDriveController(Constants.Scoring.X_CONTROLLER, Constants.Scoring.Y_CONTROLLER, Constants.Scoring.THETA_CONTROLLER);
    currentPose = poseEstimator.getEstimatedPose();
    addRequirements(poseEstimator, vision, swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    selectedGridId = GridSelector.getClosestId(vision, poseEstimator);
    tagPose = GridSelector.getTagPose2d(selectedGridId);
    nextToTagPose = new Pose2d(tagPose.getX(), tagPose.getY() + Constants.Scoring.NEXT_TO_TAG_OFFSET, tagPose.getRotation());
    futureTrajectory = AsyncTrajectory.generateTrajectory(currentPose, nextToTagPose, new ArrayList<>(), Constants.Scoring.SCORING_TRAJECTORY_CONFIG);    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    selectedGridId = GridSelector.getClosestId(vision, poseEstimator);
    if(futureTrajectory.isDone() && !isTrajectoryGenerated) {
      try{
        trajectory = futureTrajectory.get();
        isTrajectoryGenerated = true;
        aLogger.recordOutput("Scoring/MoveToGridTrajectory", trajectory);
      }
      catch(Exception Exception) {
        throw new RuntimeException("MoveToGrid unreachable block");
      }
    }
    Trajectory.State goal = trajectory.sample(Constants.Scoring.TRAJECTORY_SAMPLE_TIME);
    ChassisSpeeds chassisSpeeds = controller.calculate(currentPose, goal, nextToTagPose.getRotation());
    SwerveModuleState[] moduleStates = Constants.RobotDimensions.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
    swerve.setModuleStates(moduleStates);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(GridSelector.comparePose(currentPose, nextToTagPose)) {
      isBotAtGrid = true;
    }
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public GridLocation getSelectedGrid() {
    return GridSelector.getGridLocation(selectedGridId);
  }

  public boolean isBotAtGrid() {
    return isBotAtGrid;
  }
}
