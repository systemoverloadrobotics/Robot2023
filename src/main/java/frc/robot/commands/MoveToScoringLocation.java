// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import frc.robot.Constants;
import frc.robot.GridSelector;
import frc.robot.GridSelector.GridLocation;
import frc.robot.subsystems.IntelligentScoring.ScoringLocations;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DriveTrainPoseEstimator;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.ArmSubsystem.ArmHeight;
import frc.sorutil.path.AsyncTrajectory;
import java.util.ArrayList;
import java.util.concurrent.Future;
import java.util.logging.Logger;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** Moves bot to the desired scoring location. */
public class MoveToScoringLocation extends CommandBase {
  private final Logger logger;
  private final org.littletonrobotics.junction.Logger alogger;
  private final Swerve swerve;
  private final DriveTrainPoseEstimator poseEstimator;
  private final Vision vision;
  private final ArmSubsystem arm;
  private final Claw claw;
  private Pose2d currentPose, ScoringLocationPose;
  private final GridLocation selectedGridLocation;
  private double offsetRight, offsetLeft;
  private final ScoringLocations scoringLocation;
  private Future<Trajectory> futureTrajectory;
  private boolean isTrajectoryGenerated;
  private Trajectory trajectory;
  private HolonomicDriveController controller;
  private ArmHeight height;
  

  /**
   * Creates a new MoveToScoringLocation Command.
   */
  public MoveToScoringLocation(DriveTrainPoseEstimator poseEstimator, Swerve swerve, Vision vision, ArmSubsystem arm, Claw claw, GridLocation selectedGridLocation, ScoringLocations scoringLocation) {
    logger = Logger.getLogger(MoveToScoringLocation.class.getName());
    alogger = org.littletonrobotics.junction.Logger.getInstance();
    this.poseEstimator = poseEstimator;
    this.swerve = swerve;
    this.vision = vision;
    this.arm = arm;
    this.claw = claw;
    this.selectedGridLocation = selectedGridLocation;
    this.scoringLocation = scoringLocation;
    controller = new HolonomicDriveController(Constants.Scoring.X_CONTROLLER, Constants.Scoring.Y_CONTROLLER, Constants.Scoring.THETA_CONTROLLER);
    currentPose = poseEstimator.getEstimatedPose();
    addRequirements(poseEstimator, vision, swerve, arm, claw);
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
    //sets a new pose given the current node selected.
    switch((scoringLocation.ordinal()+1) % 3) {
      //left node
      case 0:
        ScoringLocationPose = new Pose2d(currentPose.getX() + offsetLeft, currentPose.getY(), currentPose.getRotation());
      //right node
      case 1:
        ScoringLocationPose = new Pose2d(currentPose.getX() + offsetRight, currentPose.getY(), currentPose.getRotation());
    }
    if(!GridSelector.comparePose(currentPose, ScoringLocationPose)) {
      futureTrajectory = AsyncTrajectory.generateTrajectory(currentPose, ScoringLocationPose, new ArrayList<>(), Constants.Scoring.SCORING_TRAJECTORY_CONFIG);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //move to the correct node given the grid
    if(!(futureTrajectory.isDone() && !isTrajectoryGenerated)) {
      return;
    }
    try {
      trajectory = futureTrajectory.get();
      isTrajectoryGenerated = true;
      alogger.recordOutput("Scoring/MoveToScoringLocationTrajectory", trajectory);
    }
    catch(Exception Exception) {
      throw new RuntimeException("MoveToScoringLocation unreachable block");
    }
    Trajectory.State goal = trajectory.sample(Constants.Scoring.TRAJECTORY_SAMPLE_TIME);
    ChassisSpeeds chassisSpeeds = controller.calculate(currentPose, goal, ScoringLocationPose.getRotation());
    SwerveModuleState[] moduleStates = Constants.RobotDimensions.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
    swerve.setModuleStates(moduleStates);
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