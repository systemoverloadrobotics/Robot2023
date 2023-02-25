// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.SwerveDrive;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;

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
  private Swerve swerve = new Swerve(); 
  private ArmSubsystem arm = new ArmSubsystem(); 
  private Claw claw = new Claw(); 
  
  private Command pickUpGamePieceLow = new FunctionalCommand(() -> {}, () -> {
    arm.setPosition(ArmSubsystem.ArmHeight.LOW);
    if (arm.withinRange()) {
      claw.intake();
    }
  }, (a) -> {arm.stop(); claw.stop();}, () -> false, arm, claw);
  private Command pickUpGamePieceTray = new FunctionalCommand(() -> {}, () -> {
    arm.setPosition(ArmSubsystem.ArmHeight.TRAY);
    if (arm.withinRange()) {
      claw.intake();
    }
  }, (a) -> {arm.stop(); claw.stop();}, () -> false, arm, claw);
  private Command depositGamePieceMid = new FunctionalCommand(() -> {}, () -> {
    arm.setPosition(ArmSubsystem.ArmHeight.MID);
    if (arm.withinRange()) {
      claw.outtake();
    }
  }, (a) -> {arm.stop(); claw.stop();}, () -> false, arm, claw);
  private Command depositGamePieceHigh = new FunctionalCommand(() -> {}, () -> {
    arm.setPosition(ArmSubsystem.ArmHeight.HIGH);
    if (arm.withinRange()) {
      claw.outtake();
    }
  }, (a) -> {arm.stop(); claw.stop();}, () -> false, arm, claw);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    logger = java.util.logging.Logger.getLogger(RobotContainer.class.getName());

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
