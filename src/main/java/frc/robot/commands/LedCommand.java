// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Led;
import java.util.logging.Logger;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.XboxController;


/** An example command that uses an example subsystem. */
public class LedCommand extends CommandBase {
  private final Logger logger;
  private final Led led;
  private final int red;
  private final int green;
  private final int blue;

  /**
   * Creates a new ExampleCommand.
   *
   * @param Led The subsystem used by this command.
   */
  public LedCommand(Led led, int red, int green, int blue) {
    logger = Logger.getLogger(LedCommand.class.getName());
    this.led = led;
    this.red = red;
    this.green = green;
    this.blue = blue;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(led);
    
    
   
     


    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    led.setLEDColor(red, green, blue);
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