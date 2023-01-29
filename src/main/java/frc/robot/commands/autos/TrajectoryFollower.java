// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class TrajectoryFollower extends CommandBase {

  /** Creates a new FollowTrajectory. */

  private HolonomicDriveController hController;
  private PathPlannerState state;
  private Pose2d currentPose;
  private ChassisSpeeds speeds = new ChassisSpeeds();
  private PathContainer pathConstainer;
  private PathPlannerTrajectory path;

  private final Timer timer = new Timer();

  Swerve swerve;

  public TrajectoryFollower(Swerve swerve, PathContainer pathContainer) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerve = swerve;
    this.pathContainer = pathContainer;

    path = pathContainer.getTrajectory();
    addRequirements(m_Subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    controlPID.enableContinuousInput(-Math.PI, Math.PI);
    hController =
      new HolonomicDriveController(
        new PIDController(0, 0, 0),
        new PIDController(0, 0, 0),
        controlPID
      );

    timer.stop();
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var currTime = timer.get();
    state = (PathPlannerState) path.sample(currTime);
    currentPose = m_Subsystem.getPose();
    speeds = hController.calculate(currentPose, state, state.holonomicRotation);
    m_Subsystem.updateStates(
      m_Subsystem.getKinematics().toSwerveModuleStates(speeds)
    );
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Subsystem.updateStates(
      m_Subsystem.getKinematics().toSwerveModuleStates(new ChassisSpeeds())
    );
    timer.stop();
  }
  // Returns true when the command should end.

}
