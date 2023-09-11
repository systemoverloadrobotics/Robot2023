package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj.SerialPort;

public class MoveStraightAuton extends CommandBase {
    private final Swerve swerve;   
    private final double distance;

    public MoveStraightAuton(Swerve swerve, double distance) {
        this.swerve = swerve;
        this.distance = distance;
        addRequirements(swerve);
        
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        if (swerve.getDisplacementZ() > distance) { 
            end(false);
        }
        else {
            swerve.setDrivebaseWheelVectors(1, 0, 0, false, false);
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
