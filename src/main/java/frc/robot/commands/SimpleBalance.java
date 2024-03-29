package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class SimpleBalance extends CommandBase {
    private final Swerve swerve;
    private boolean onRamp;
    private double prevPitch;

    public SimpleBalance(Swerve swerve) {
        this.swerve = swerve;
        onRamp = false;
        prevPitch = swerve.getPitch();

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        onRamp = false;
        prevPitch = swerve.getPitch();
        super.initialize();
    }

    @Override
    public void execute() {
        System.out.println(swerve.getPitch());
        if (!onRamp) {
            swerve.setDrivebaseWheelVectors(Constants.Swerve.SWERVE_MAX_AUTO_SPEED, 0, 0, false, false);
            if (Math.abs(swerve.getPitch()) > 12) { // arbitrary, make into const
                onRamp = true;
            }        
        } else {
            if (Math.abs(swerve.getPitch() - prevPitch) > 0.1) {
                swerve.lock();
                this.cancel();
            } else {
                swerve.setDrivebaseWheelVectors(Constants.Swerve.SWERVE_MAX_PRECISION_SPEED, 0, 0, false, false);
            }

        }
        prevPitch = swerve.getPitch();
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
