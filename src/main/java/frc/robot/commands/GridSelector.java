// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Swerve;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrainPoseEstimator;
import frc.robot.subsystems.Vision;
import frc.sorutil.path.AsyncTrajectory;
import java.util.logging.Logger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class GridSelector extends CommandBase {
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
  public GridSelector(DriveTrainPoseEstimator poseEstimator, Swerve swerve, Vision vision) {
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
    targetPose = vision.getPose2d(getClosestId());
    Trajectory tagTrajectory = (Trajectory) AsyncTrajectory.generateTrajectory(currentPose, targetPose, null, Constants.Scoring.SCORING_TRAJECTORY_CONFIG);
    
  }

  //gives closest apriltag to the bot
  public int getClosestId() {
    var allianceTargets = Constants.Scoring.TARGETS_PER_ALLIANCE.get(DriverStation.getAlliance());
    double closestDistance = -1;
    int closestId = 0;
    //finds the closest april tag to the bot
    for(int id : allianceTargets) {
      double tempIdDistance = distanceFormula(currentPose.getX(), currentPose.getY(), vision.getPose2d(id).getX(), vision.getPose2d(id).getY());
      if(tempIdDistance > closestDistance) {
        closestDistance = tempIdDistance;
        closestId = id;
      }
    }
    //checks if the closest april tag if within the minimum distance
    targetPose = vision.getPose2d(closestId);
    if (distanceFormula(currentPose.getX(), currentPose.getY(), targetPose.getX(), targetPose.getY()) > Constants.Scoring.MIN_AUTOMOVE_DISTANCE) {
          return -1;
    }
    return closestId;
  }

  public GridLocation getGridLocation(int id) {
    switch (id) {
      case 1:
        return GridLocation.RIGHT;
      case 2:
        return GridLocation.MIDDLE;
      case 3:
        return GridLocation.LEFT;
      case 6:
        return GridLocation.RIGHT;
      case 7:
        return GridLocation.MIDDLE;
      case 8:
        return GridLocation.LEFT;
      default:
        return null;
    }
  }
  public static enum GridLocation {
    LEFT, MIDDLE, RIGHT
  }

  
  public double distanceFormula(double x1, double y1, double x2, double y2) {
    return Math.sqrt((Math.pow(y2 - y1, 2)) + (Math.pow(x2 - x1, 2)));
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
