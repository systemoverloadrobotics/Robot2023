package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.sorutil.SorMath;

public class SwerveDrive extends CommandBase {
  private final Swerve swerve;
  private DoubleSupplier xSupplier, ySupplier, rotationSupplier;
  private SlewRateLimiter xLimiter, yLimiter, rotationLimiter;

  public SwerveDrive(Swerve swerve, DoubleSupplier xSupplier, DoubleSupplier ySupplier,
      DoubleSupplier rotationSupplier) {
    this.swerve = swerve;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.rotationSupplier = rotationSupplier;

    xLimiter = new SlewRateLimiter(Constants.Swerve.SWERVE_MAX_SPEED);
    yLimiter = new SlewRateLimiter(Constants.Swerve.SWERVE_MAX_SPEED);
    rotationLimiter = new SlewRateLimiter(Constants.Swerve.SWERVE_ROTATION_MAX_SPEED);
    addRequirements(swerve);
  }

  private double cleanAndScaleInput(double input, SlewRateLimiter limiter, double speedScaling) {
    input = (Math.abs(input) > Constants.Swerve.SWERVE_DEADBAND) ? input : 0;
    SmartDashboard.putNumber("input", input); 
    input = SorMath.signedSquare(input);
    input = limiter.calculate(input);
    input *= speedScaling;

    return input;
  }

  // Called at 50hz while the command is scheduled.
  @Override
  public void execute() {
    // get joystick inputs and clean/scale them
    double xSpeed = cleanAndScaleInput(xSupplier.getAsDouble(), xLimiter,
        Constants.Swerve.SWERVE_MAX_SPEED);
    double ySpeed = cleanAndScaleInput(ySupplier.getAsDouble(), yLimiter,
        Constants.Swerve.SWERVE_MAX_SPEED);
    double rotationSpeed = cleanAndScaleInput(rotationSupplier.getAsDouble(), rotationLimiter,
        Constants.Swerve.SWERVE_ROTATION_MAX_SPEED);

    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed,
        rotationSpeed, swerve.getRotation2d());
    // Calculate swerve module states using the desired state of the robot chassis.
    SwerveModuleState[] moduleState =
        Constants.RobotDimensions.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleState,
        Constants.Swerve.SWERVE_MAX_SPEED);
    swerve.setModuleStates(moduleState);
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
