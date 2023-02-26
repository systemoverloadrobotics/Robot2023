// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.commands.SwerveDrive;
import frc.robot.commands.autos.AutoPaths;
import frc.robot.commands.autos.AutoSelector;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
  private AutoSelector autoSelector = new AutoSelector(swerve);
 
  private Led led = new Led();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    logger = java.util.logging.Logger.getLogger(RobotContainer.class.getName());
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
        () -> Constants.Input.SWERVE_Y_INPUT.get().getAsDouble(), Constants.Input.SWERVE_ROTATION_INPUT.get()));

      Constants.Input.LED_TRIGGER_PURPLE.get().whenHeld(ledCommandPurple);
      Constants.Input.LED_TRIGGER_YELLOW.get().whenHeld(ledCommandYellow);
  }



  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return autoSelector.getAuto();
  }
}