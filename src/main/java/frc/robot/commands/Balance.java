package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class Balance extends CommandBase {
  private final Swerve swerve;
  private final PIDController pid;
  private boolean onRamp;

  public Balance(Swerve swerve) {
    this.swerve = swerve;
    pid = new PIDController(0, 0, 0);
    onRamp = false;

    addRequirements(swerve);
  }

  @Override
  public void execute() {
    // A bit janky
    // ROBOT NEEDS TO BE HEADED TO RAMP!!!!!!!!!!
    if (!onRamp) {
      swerve.setDrivebaseWheelVectors(0, 0.5, 0, false);
      if (Math.abs(swerve.getPitch()) > 8) { // arbitrary, make into const
        onRamp = true;
      }
    } else {
      double out = pid.calculate(swerve.getPitch(), 0);
      if (Math.abs(swerve.getPitch()) < 2) { // arbitrary, make into const
        swerve.lock();
      } else {
        swerve.setDrivebaseWheelVectors(0, out, 0, false);
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
