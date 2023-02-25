// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.MoveToGrid;
import frc.robot.commands.MoveToHumanPlayer;
import frc.robot.commands.MoveToScoringLocation;
import frc.robot.commands.SwerveDrive;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.IntelligentScoring.ScoringLocations;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DriveTrainPoseEstimator;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.Led;

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
  private Led led;


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    logger = java.util.logging.Logger.getLogger(RobotContainer.class.getName());
    poseEstimator = new DriveTrainPoseEstimator();
    vision = new Vision();
    arm = new ArmSubsystem();
    claw = new Claw();
    led = new Led();
    swerve = new Swerve();
    // Configure the button bindings
    configureButtonBindings();
  }

  private final Command ledCommandPurple = new RunCommand(() -> {
    led.setLEDColor(true);
  }, led);
  private final Command ledCommandYellow = new RunCommand(() -> {
    led.setLEDColor(false);
  }, led);

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
    Constants.Input.POSITION_TO_CLOSEST_GRID.get().onTrue(new MoveToGrid(poseEstimator, swerve, vision));
    Constants.Input.POSITION_TO_HUMAN_PLAYER.get().onTrue(new MoveToHumanPlayer(swerve, poseEstimator));

    Constants.Input.UPPER_LEFT_CONE.get().onTrue(new MoveToScoringLocation(poseEstimator, swerve, vision, arm, claw, GridSelector.getSelectedGrid(), ScoringLocations.UPPER_LEFT_CONE));
    Constants.Input.UPPER_MIDDLE_CUBE.get().onTrue(new MoveToScoringLocation(poseEstimator, swerve, vision, arm, claw, GridSelector.getSelectedGrid(), ScoringLocations.UPPER_MIDDLE_CUBE));
    Constants.Input.UPPER_RIGHT_CONE.get().onTrue(new MoveToScoringLocation(poseEstimator, swerve, vision, arm, claw, GridSelector.getSelectedGrid(), ScoringLocations.UPPER_RIGHT_CONE));
    Constants.Input.MIDDLE_LEFT_CONE.get().onTrue(new MoveToScoringLocation(poseEstimator, swerve, vision, arm, claw, GridSelector.getSelectedGrid(), ScoringLocations.MIDDLE_LEFT_CONE));
    Constants.Input.MIDDLE_MIDDLE_CUBE.get().onTrue(new MoveToScoringLocation(poseEstimator, swerve, vision, arm, claw, GridSelector.getSelectedGrid(), ScoringLocations.MIDDLE_MIDDLE_CUBE));
    Constants.Input.MIDDLE_RIGHT_CONE.get().onTrue(new MoveToScoringLocation(poseEstimator, swerve, vision, arm, claw, GridSelector.getSelectedGrid(), ScoringLocations.MIDDLE_RIGHT_CONE));
    Constants.Input.HYBRID_LEFT.get().onTrue(new MoveToScoringLocation(poseEstimator, swerve, vision, arm, claw, GridSelector.getSelectedGrid(), ScoringLocations.HYBRID_LEFT));
    Constants.Input.HYBRID_MIDDLE.get().onTrue(new MoveToScoringLocation(poseEstimator, swerve, vision, arm, claw, GridSelector.getSelectedGrid(), ScoringLocations.HYBRID_MIDDLE));
    Constants.Input.HYBRID_RIGHT.get().onTrue(new MoveToScoringLocation(poseEstimator, swerve, vision, arm, claw, GridSelector.getSelectedGrid(), ScoringLocations.HYBRID_RIGHT));

      Constants.Input.LED_TRIGGER_PURPLE.get().whileTrue(ledCommandPurple);
      Constants.Input.LED_TRIGGER_YELLOW.get().whileTrue(ledCommandYellow);
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