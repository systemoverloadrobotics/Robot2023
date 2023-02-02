package frc.robot.commands.autos;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

/**
 * Run auto commands
 */
//TODO: enter PID values
public class AutoPaths {
    public static PPSwerveControllerCommand getPathCommand(Swerve swerve, String path) {
        PathPlannerTrajectory trajectory = PathPlanner.loadPath(path, Constants.Swerve.SWERVE_MAX_SPEED, Constants.Swerve.SWERVE_MAX_ACCELERATION);
        PPSwerveControllerCommand command = new PPSwerveControllerCommand(
            trajectory,
            swerve::getOdometryPose, 
            Constants.RobotDimensions.SWERVE_DRIVE_KINEMATICS,
            Constants.Auto.X_PID_CONTROLLER,
            Constants.Auto.Y_PID_CONTROLLER, 
            Constants.Auto.ROT_PID_CONTROLLER,
            swerve::setModuleStates, 
            swerve
        );
        return command;
    }

    private static Command createAutoCommand(Swerve swerve, ScoringPosition goal, PieceCount piece, boolean balance) {
        String alliance = DriverStation.getAlliance().equals(Alliance.Red) ? "red" : "blue";
        String basePathFileName = "auto_" + alliance + "_" + getDSPosition() + "_";
        // Do arm stuff here for config from ScoringPosition
        SequentialCommandGroup command = new SequentialCommandGroup();
        command.addCommands(); // TODO: arm command for score
        switch (piece) {
            case ONE:
                command.addCommands(getPathCommand(swerve, basePathFileName + "one_A"));
                if (balance) {
                    command.addCommands(getPathCommand(swerve, "auto_balance_prep" + alliance));
                    command.addCommands(); // TODO: Balance command
                }
                break;
            case TWO:
                command.addCommands(getPathCommand(swerve, basePathFileName + "two_A"));
                command.addCommands(); // TODO: arm command for pickup
                command.addCommands(getPathCommand(swerve, basePathFileName + "two_B"));
                command.addCommands(); // TODO: arm command for score
        }
        return command;
    }

    private static String getDSPosition() {
        switch (DriverStation.getLocation()) {
            case 1:
                return "left";
            case 2:
                return "middle";
            case 3:
                return "right";
            default:
                return null;
        }
    }

    public enum PieceCount {
        ONE("one"),
        TWO("two");

        String id;

        PieceCount(String str) {
            id = str;
        }

        @Override
        public String toString() {
            return id;
        }
    }
    
    public enum ScoringPosition {
        HIGH("high"),
        MID("mid");

        String id;
        
        ScoringPosition(String str) {
            id = str;
        }

        @Override
        public String toString() {
            return id;
        }
    }
}
