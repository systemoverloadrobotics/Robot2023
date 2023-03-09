package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Claw;


public class PickUpGamePiece extends CommandBase {
  private ArmSubsystem arm;
  private Claw claw;
  private ArmSubsystem.ArmHeight height;

  public PickUpGamePiece(ArmSubsystem arm, Claw claw, ArmSubsystem.ArmHeight height) {
    this.arm = arm;
    this.claw = claw;
    this.height = height;

    addRequirements(arm, claw);
  }

  // Called at 50hz while the command is scheduled.
  @Override
  public void execute() {
    arm.setPosition(height);
    if (arm.withinRange()) {
      claw.intake();
    }
  }

  // Called once when the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.stop();
    claw.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
