package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Swerve;

public class IntakeClaw extends CommandBase {
  private final Claw claw;

  public IntakeClaw(Claw claw) {
    this.claw = claw;

    addRequirements(claw);
  }

  @Override
  public void execute() {
    claw.intake();
  }

  // Called once when the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    claw.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
