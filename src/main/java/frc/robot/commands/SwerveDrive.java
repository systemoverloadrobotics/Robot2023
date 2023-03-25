package frc.robot.commands;

import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.sorutil.SorMath;


public class SwerveDrive extends CommandBase {
    protected final Swerve swerve;
    protected SlewRateLimiter xLimiter, yLimiter, rotationLimiter;

    private final DoubleSupplier xSupplier, ySupplier, rotationSupplier;

    public SwerveDrive(Swerve swerve, DoubleSupplier xSupplier, DoubleSupplier ySupplier,
            DoubleSupplier rotationSupplier) {
        this.swerve = swerve;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.rotationSupplier = rotationSupplier;

        xLimiter = new SlewRateLimiter(Constants.Swerve.SWERVE_MAX_SPEED, 0.5, 0);
        yLimiter = new SlewRateLimiter(Constants.Swerve.SWERVE_MAX_SPEED, 0.5, 0);
        rotationLimiter = new SlewRateLimiter(Constants.Swerve.SWERVE_ROTATION_MAX_SPEED);
        addRequirements(swerve);
    }

    protected double cleanAndScaleInput(double input, SlewRateLimiter limiter, double speedScaling) {
        input = (Math.abs(input) > Constants.Swerve.SWERVE_DEADBAND) ? input : 0;
        input = SorMath.signedSquare(input);
        input = limiter.calculate(input);
        input *= speedScaling;

        return input;
    }

    // Called at 50hz while the command is scheduled.
    @Override
    public void execute() {
        // get joystick inputs and clean/scale them
        double xSpeed = cleanAndScaleInput(xSupplier.getAsDouble(), xLimiter, Constants.Swerve.SWERVE_MAX_SPEED);
        double ySpeed = cleanAndScaleInput(ySupplier.getAsDouble(), yLimiter, Constants.Swerve.SWERVE_MAX_SPEED);
        double rotationSpeed = cleanAndScaleInput(rotationSupplier.getAsDouble(), rotationLimiter,
                Constants.Swerve.SWERVE_ROTATION_MAX_SPEED);
        swerve.setDrivebaseWheelVectors(xSpeed, ySpeed, rotationSpeed, true, false);
    }

    protected void executeLogging(double xSpeed, double ySpeed, double rotationSpeed) {
        Logger.getInstance().recordOutput("SwerveDrive/xSpeed", xSpeed);
        Logger.getInstance().recordOutput("SwerveDrive/ySpeed", ySpeed);
        SmartDashboard.putNumber("rspeed", rotationSpeed);
        SmartDashboard.putNumber("rotation 2d", swerve.getRotation2d().getDegrees());
        Logger.getInstance().recordOutput("SwerveDrive/rotation", swerve.getRotation2d().getDegrees());
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
