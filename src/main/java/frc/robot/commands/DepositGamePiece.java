package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Swerve;
import frc.sorutil.SorMath;

public class DepositGamePiece extends CommandBase {
  private ArmSubsystem arm;
  private Claw claw;
  private ArmSubsystem.ArmHeight height;

  public DepositGamePiece(ArmSubsystem arm, Claw claw, ArmSubsystem.ArmHeight height) {
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
