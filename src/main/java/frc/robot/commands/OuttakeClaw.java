package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class OuttakeClaw extends CommandBase {
    private final Claw claw;

    public OuttakeClaw(Claw claw) {
        this.claw = claw;

        addRequirements(claw);
    }

    @Override
    public void execute() {
        claw.outtake();
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
