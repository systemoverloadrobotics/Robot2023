package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class FinetuneArm extends CommandBase {
    private final ArmSubsystem arm;
    private final DoubleSupplier udSupplier;
    private final DoubleSupplier fbSupplier;
  
    public FinetuneArm(ArmSubsystem arm, DoubleSupplier udSupplier, DoubleSupplier fbSupplier) {
      this.arm = arm;
      this.udSupplier = udSupplier;
      this.fbSupplier = fbSupplier;

      addRequirements(arm);
    }
  
    @Override
    public void execute() {
      if (udSupplier.getAsDouble() > 0.1 || fbSupplier.getAsDouble() > 0.1) {
        Double positionFB = arm.getIntendedPosition().getFirst();
        Double positionUD = arm.getIntendedPosition().getSecond();
        Pair<Double, Double> newArmPosition = new Pair<>(positionFB + (0.025 * fbSupplier.getAsDouble()), positionUD + (0.025 * udSupplier.getAsDouble()));
        arm.setPosition(newArmPosition);
      }
    }
  
    // Called once when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      arm.stop();
    }
  
    @Override
    public boolean isFinished() {
      return false;
    }
  }
  
