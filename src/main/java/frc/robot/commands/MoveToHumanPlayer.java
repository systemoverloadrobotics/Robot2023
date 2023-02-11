// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.concurrent.Future;
import java.util.logging.Logger;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.GridSelector;
import frc.robot.subsystems.DriveTrainPoseEstimator;
import frc.robot.subsystems.Swerve;
import frc.sorutil.path.AsyncTrajectory;

/** An example command that uses an example subsystem. */
public class MoveToHumanPlayer extends CommandBase {
  private final Logger logger;
  private final Swerve swerve;
  private final DriveTrainPoseEstimator poseEstimator;
  private final HolonomicDriveController controller;
  private Future<Trajectory> futureTrajectory;
  private Trajectory trajectory;
  private Pose2d currentPose;
  private Pose2d HumanPlayerTagPose;
  private boolean isTrajectoryGenerated;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public MoveToHumanPlayer(Swerve swerve, DriveTrainPoseEstimator poseEstimator) {
    logger = Logger.getLogger(MoveToHumanPlayer.class.getName());
    this.swerve = swerve;
    this.poseEstimator = poseEstimator;
    currentPose = poseEstimator.getEstimatedPose();
    controller = new HolonomicDriveController(Constants.Scoring.X_CONTROLLER, Constants.Scoring.Y_CONTROLLER, Constants.Scoring.THETA_CONTROLLER);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve, poseEstimator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    HumanPlayerTagPose = GridSelector.getTagPose2d(getHumanPlayerTag());
    futureTrajectory = AsyncTrajectory.generateTrajectory(currentPose, HumanPlayerTagPose, new ArrayList<>(), Constants.Scoring.SCORING_TRAJECTORY_CONFIG);
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
          throw new RuntimeException("MoveToHumanPlayer unreachable block");
        }
      }
      Trajectory.State goal = trajectory.sample(Constants.Scoring.TRAJECTORY_SAMPLE_TIME);
      ChassisSpeeds chassisSpeeds = controller.calculate(currentPose, goal, HumanPlayerTagPose.getRotation());
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

  //returns tag given bot alliance
  private int getHumanPlayerTag() {
    switch(DriverStation.getAlliance()) {
        case Blue:
            return 4;
        case Red:
            return 5;
        case Invalid:
            //error with alliance system, should do a DS update
        default:
            return -1; //should never occur
    }
  }
}