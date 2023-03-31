package frc.robot.commands;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class SlightlyMoreComplexBalance extends CommandBase {
    private final Swerve swerve;
    private boolean onRamp;
    private double prevPitch;
    private LinearFilter filter;

    public SlightlyMoreComplexBalance(Swerve swerve) {
        this.swerve = swerve;
        onRamp = false;
        prevPitch = swerve.getPitch();
        filter = LinearFilter.movingAverage(5);
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        onRamp = false;
        prevPitch = swerve.getPitch();
        filter = LinearFilter.movingAverage(5);
        super.initialize();
    }

    @Override
    public void execute() {
        var pitchFiltered = filter.calculate(swerve.getPitch());
        if (!onRamp) {
            swerve.setDrivebaseWheelVectors(Constants.Swerve.SWERVE_MAX_AUTO_SPEED, 0, 0, false, false);
            if (Math.abs(swerve.getPitch()) > 12) { // arbitrary, make into const
                onRamp = true;
            }        
        } else {
            if (pitchFiltered > 2) {
                swerve.setDrivebaseWheelVectors(Constants.Swerve.SWERVE_MAX_PRECISION_SPEED, 0, 0, false, false);
            } else if (pitchFiltered < -2) {
                swerve.setDrivebaseWheelVectors(-Constants.Swerve.SWERVE_MAX_PRECISION_SPEED, 0, 0, false, false);
            } else {
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
