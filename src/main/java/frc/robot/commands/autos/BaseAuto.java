package frc.robot.commands.autos;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

/**
 * Run auto commands
 */
//TODO: enter PID values
public class BaseAuto extends SequentialCommandGroup {
    public Swerve swerve;
    public static final ProfiledPIDController profiledthetaController =
        new ProfiledPIDController(0,0,0,Constants.Auto.SWERVE_PID_CONSTRAINTS );
    public static final PIDController thetaController =
        new PIDController(0, 0, 0);

   
    public BaseAuto(Swerve swerve) {
        this.swerve = swerve;
        addRequirements(swerve);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    /**
     * Creates a SwerveControllerCommand from a Trajectory
     *
     * @param trajectory Trajectory to run
     * @return A SwerveControllerCommand for the robot to move
     */
    public SwerveControllerCommand baseSwerveCommand(Trajectory trajectory) {
        SwerveControllerCommand command = new SwerveControllerCommand(trajectory, swerve::getPose,
            Constants.RobotDimensions.SWERVE_DRIVE_KINEMATICS,
            new PIDController(0, 0, 0),
            new PIDController(0, 0, 0), profiledthetaController,
            swerve::setModuleStates, swerve);
        return command;
    }

    /**
     * Creates a SwerveController Command using a Path Planner Trajectory
     *
     * @param trajectory a Path Planner Trajectory
     * @return A SwerveControllerCommand for the robot to move
     */
    public PPSwerveControllerCommand baseSwerveCommand(PathPlannerTrajectory trajectory) {
        PPSwerveControllerCommand command = new PPSwerveControllerCommand(trajectory,
            swerve::getPose, Constants.RobotDimensions.SWERVE_DRIVE_KINEMATICS,
            new PIDController(0, 0, 0),
            new PIDController(0, 0, 0), thetaController,
            swerve::setModuleStates, swerve);
        return command;
    }
}
