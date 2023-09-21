package frc.robot.commands;

import org.littletonrobotics.junction.Logger;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj.SerialPort;

public class MoveStraightAuton extends CommandBase {
    private final Swerve swerve;   
    private final double time;
    private final double xSpeed;
    private final double ySpeed;
    public double currentElapsed;
    private boolean isFinished;

    public MoveStraightAuton(Swerve swerve, double time, double xSpeed, double ySpeed) {
        this.swerve = swerve;
        // seconds -> robot cycles
        this.time = time;
        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        this.currentElapsed = 0;
        addRequirements(swerve);
        
    }

    @Override
    public void initialize() {
        super.initialize();
        
    }

    @Override
    public void execute() {
        Logger.getInstance().recordOutput("TimeElapsed", currentElapsed);
        Logger.getInstance().recordOutput("xspeedreq", xSpeed);
        if (currentElapsed > time) { 
            isFinished = true;
        }
        else {
            swerve.setDrivebaseWheelVectors(xSpeed, ySpeed, 0, false, false);
        }
        currentElapsed += 0.02;
    }

    // Called once when the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        swerve.stopModules();
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }
}
