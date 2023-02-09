// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Arm;
import frc.robot.GridSelector.GridLocation;
import frc.robot.commands.MoveToGrid;
import frc.robot.commands.MoveToScoringLocation;
import frc.robot.commands.SwerveDrive;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DriveTrainPoseEstimator;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  @SuppressWarnings("unused")
  private final java.util.logging.Logger logger;

  // The robot's subsystems and commands are defined here...
  private Swerve swerve; 
  private DriveTrainPoseEstimator poseEstimator;
  private Vision vision;
  private ArmSubsystem arm;
  private Claw claw;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    logger = java.util.logging.Logger.getLogger(RobotContainer.class.getName());
    poseEstimator = new DriveTrainPoseEstimator();
    vision = new Vision();
    arm = new ArmSubsystem();
    claw = new Claw();
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    swerve.setDefaultCommand(new SwerveDrive(swerve, () -> -Constants.Input.SWERVE_X_INPUT.get().getAsDouble(),
        () -> -Constants.Input.SWERVE_Y_INPUT.get().getAsDouble(), Constants.Input.SWERVE_ROTATION_INPUT.get()));

    //scoring
    Constants.Input.UPPER_LEFT_CONE.get().onTrue(new MoveToScoringLocation(poseEstimator, swerve, vision, arm, claw, GridSelector.getSelectedGrid(), 1));
    Constants.Input.UPPER_MIDDLE_CUBE.get().onTrue(new MoveToScoringLocation(poseEstimator, swerve, vision, arm, claw, GridSelector.getSelectedGrid(), 2));
    Constants.Input.UPPER_RIGHT_CONE.get().onTrue(new MoveToScoringLocation(poseEstimator, swerve, vision, arm, claw, GridSelector.getSelectedGrid(), 3));
    Constants.Input.MIDDLE_LEFT_CONE.get().onTrue(new MoveToScoringLocation(poseEstimator, swerve, vision, arm, claw, GridSelector.getSelectedGrid(), 4));
    Constants.Input.MIDDLE_LEFT_CONE.get().onTrue(new MoveToScoringLocation(poseEstimator, swerve, vision, arm, claw, GridSelector.getSelectedGrid(), 5));
    Constants.Input.MIDDLE_LEFT_CONE.get().onTrue(new MoveToScoringLocation(poseEstimator, swerve, vision, arm, claw, GridSelector.getSelectedGrid(), 6));
    Constants.Input.HYBRID_LEFT.get().onTrue(new MoveToScoringLocation(poseEstimator, swerve, vision, arm, claw, GridSelector.getSelectedGrid(), 7));
    Constants.Input.HYBRID_MIDDLE.get().onTrue(new MoveToScoringLocation(poseEstimator, swerve, vision, arm, claw, GridSelector.getSelectedGrid(), 8));
    Constants.Input.HYBRID_RIGHT.get().onTrue(new MoveToScoringLocation(poseEstimator, swerve, vision, arm, claw, GridSelector.getSelectedGrid(), 9));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
