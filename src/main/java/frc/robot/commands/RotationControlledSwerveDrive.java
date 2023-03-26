package frc.robot.commands;

import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.controller.ProfiledPIDController;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.sorutil.SorMath;

/**
 * RotationControlledSwerveDrive is similar to the base SwerveDrive class, except that instead of a rotation scalar
 * being supplied, instead a heading vector is supplied (in degrees). The orientation of the heading vector is zero
 * degrees when it is pointed away from the driver station.
 * 
 * If -1 is supplied to the heading supplier, it will instead attempt to maintain the last heading supplied to the
 * command, or if one has not yet been supplied, the heading of the robot when the command was first scheduled.
 */
public class RotationControlledSwerveDrive extends SwerveDrive {
    private final DoubleSupplier xSupplier, ySupplier, headingSupplier;
    private double lastHeading;

    private final ProfiledPIDController angleController =
            new ProfiledPIDController(0.5, 0, 0, Constants.Auto.SWERVE_ROTATION_PID_CONSTRAINTS);

    public RotationControlledSwerveDrive(Swerve swerve, DoubleSupplier xSupplier, DoubleSupplier ySupplier,
            DoubleSupplier headingSupplier) {
        super(swerve, xSupplier, ySupplier, null);

        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.headingSupplier = headingSupplier;

        angleController.setTolerance(Constants.Swerve.SWERVE_ROTATION_TOLERANCE);
        angleController.enableContinuousInput(0, 360);
    }

    private double currentHeading() {
        return swerve.getRotation2d().getDegrees();
    }

    @Override
    public void initialize() {
        super.initialize();
        
        angleController.reset(currentHeading(), 0);
        lastHeading = currentHeading();
    }

    @Override
    public void execute() {
        double xSpeed = cleanAndScaleInput(xSupplier.getAsDouble(), xLimiter, Constants.Swerve.SWERVE_MAX_SPEED);
        double ySpeed = cleanAndScaleInput(ySupplier.getAsDouble(), yLimiter, Constants.Swerve.SWERVE_MAX_SPEED);
        
        double desiredHeading = headingSupplier.getAsDouble() != -1 ? headingSupplier.getAsDouble() : lastHeading;

        double rotSpeed = angleController.calculate(currentHeading(), desiredHeading);
        swerve.setDrivebaseWheelVectors(xSpeed, ySpeed, rotSpeed, true, false);

        super.executeLogging(xSpeed, ySpeed, rotSpeed);

        Logger.getInstance().recordOutput("SwerveDrive/HeadingTarget", desiredHeading);
        Logger.getInstance().recordOutput("SwerveDrive/HeadingError", angleController.getPositionError());
    }

    /**
     * Snaps the input X, Y from a joystick into a heading value with 4 divisions of a circle. Will also check that the
     * stick is pushed far enough to register as an input, and if it doesn't, this will return -1.
     * 
     * @return the angle that is closest to the position of the stick, or -1 if it doesn't surpass a deadband threshold.
     */
    public static int snapThresholdJoystickAxis(double x, double y) {
        // Find the polar coordinates of the vector they create.
        double[] stickPolar = SorMath.cartesianToPolar(x, y);

        // If the stick is moved less than the snapping deadband, leave the rotation as is.
        if (stickPolar[0] < Constants.Swerve.SWERVE_SNAPPING_DEADBAND) {
            return -1;
        }

        // Make the vector theta positive no matter what
        if (stickPolar[1] < 0) {
            stickPolar[1] = 360 + stickPolar[1];
        }

        // Turn the vector so that the "zero" position of the stick is straight ahead, capped to 360deg.
        stickPolar[1] = stickPolar[1] - 90.0;
        if (stickPolar[1] < 0) {
            stickPolar[1] = 360 + stickPolar[1];
        }

        int thetaSnapped = SorMath.circleSnappingDegrees(stickPolar[1], 4);

        return thetaSnapped;
    }
}
