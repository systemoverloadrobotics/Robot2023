package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.IntelligentScoring;
import frc.robot.subsystems.ArmSubsystem.ArmHeight;

public class DepositGamePiece extends CommandBase {
  private ArmSubsystem arm;
  private Claw claw;
  private ArmSubsystem.ArmHeight height;
  private IntelligentScoring.ScoringLocations scoringLocation;

  public DepositGamePiece(ArmSubsystem arm, Claw claw, ArmSubsystem.ArmHeight height, IntelligentScoring.ScoringLocations scoringLocation) {
    this.arm = arm;
    this.claw = claw;
    this.height = height;
    this.scoringLocation = scoringLocation;

    addRequirements(arm, claw);
  }

  // Called at 50hz while the command is scheduled.
  @Override
  public void execute() {
    switch (scoringLocation) {
      case UPPER_LEFT_CONE, UPPER_RIGHT_CONE:
        height = ArmHeight.HIGH_CONE;
      case UPPER_MIDDLE_CUBE:
        height = ArmHeight.HIGH_CUBE;
      case MIDDLE_LEFT_CONE, MIDDLE_RIGHT_CONE:
        height = ArmHeight.MID_CONE;
      case MIDDLE_MIDDLE_CUBE:
        height = ArmHeight.MID_CUBE;
      case HYBRID_LEFT, HYBRID_MIDDLE, HYBRID_RIGHT:
        height = ArmHeight.LOW;
    }
    arm.setPosition(height.getCoordinates());
    if(arm.withinRange(height.getCoordinates(), 0.25, 0.25)) {
      claw.outtake();
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
