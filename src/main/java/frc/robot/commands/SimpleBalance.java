package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class SimpleBalance extends CommandBase {
  private final Swerve swerve;
  private boolean onRamp;

  public SimpleBalance(Swerve swerve) {
    this.swerve = swerve;
    onRamp = false;

    addRequirements(swerve);
  }

  @Override
  public void execute() {
    // A bit janky
    // ROBOT NEEDS TO BE HEADED TO RAMP!!!!!!!!!!
    if (!onRamp) {
      swerve.setModuleStates(0, 0.5, 0, false);
      if (Math.abs(swerve.getPitch()) > 8) { // arbitrary, make into const
        onRamp = true;
      }
    } else {
      if (swerve.getPitch() > 2) { // arbitrary, make into const
        swerve.setModuleStates(0, 0.25, 0, false);
      }
      else if (swerve.getPitch() < 2) { // arbitrary, make into const
        swerve.setModuleStates(0, -0.25, 0, false);
      }
      else {
        swerve.lock();
      }
    }
  }

  // Called once when the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.stopModules();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
