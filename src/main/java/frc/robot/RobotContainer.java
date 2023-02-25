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
  private Swerve swerve = new Swerve(); 
  private ArmSubsystem arm = new ArmSubsystem(); 
  private Claw claw = new Claw(); 
  private Led led = new Led();
  
  private Command pickUpGamePieceLow = new FunctionalCommand(() -> {}, () -> arm.setPosition(ArmSubsystem.ArmHeight.LOW), 
    (a) -> arm.stop(), () -> arm.withinRange(), arm);
  private Command pickUpGamePieceTray = new FunctionalCommand(() -> {}, () -> arm.setPosition(ArmSubsystem.ArmHeight.TRAY), 
    (a) -> arm.stop(), () -> arm.withinRange(), arm);
  private Command depositGamePieceMid = new FunctionalCommand(() -> {}, () -> arm.setPosition(ArmSubsystem.ArmHeight.MID), 
    (a) -> arm.stop(), () -> arm.withinRange(), arm);
  private Command depositGamePieceHigh = new FunctionalCommand(() -> {}, () -> arm.setPosition(ArmSubsystem.ArmHeight.HIGH), 
    (a) -> arm.stop(), () -> arm.withinRange(), arm);
  private Command intakeClaw = new FunctionalCommand(() -> {}, () -> claw.intake(), 
    (a) -> claw.stop(), () -> arm.withinRange(), claw);
  private Command outtakeClaw = new FunctionalCommand(() -> {}, () -> claw.outtake(), 
    (a) -> claw.stop(), () -> arm.withinRange(), claw);
  private Command stowArm = new FunctionalCommand(() -> {}, () -> arm.setPosition(ArmSubsystem.ArmHeight.STOW), 
    (a) -> arm.stop(), () -> arm.withinRange(), claw);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    logger = java.util.logging.Logger.getLogger(RobotContainer.class.getName());

    // Configure the button bindings
    configureButtonBindings();

    // Configure arm zero
    configureArm();
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
        () -> Constants.Input.SWERVE_Y_INPUT.get().getAsDouble(), Constants.Input.SWERVE_ROTATION_INPUT.get()));

      Constants.Input.LED_TRIGGER_PURPLE.get().whenHeld(ledCommandPurple);
      Constants.Input.LED_TRIGGER_YELLOW.get().whenHeld(ledCommandYellow);
  }

  private void configureArm() {
    arm.zeroElevator();
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