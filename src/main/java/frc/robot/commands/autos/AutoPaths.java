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
            Constants.Auto.DEG_PID_CONTROLLER,
            swerve::setModuleStates, 
            swerve
        );
        return command;
    }

    private static Command createAutoCommand(Swerve swerve, ScoringPosition goal, PieceCount piece, boolean balance) {
        String alliance = DriverStation.getAlliance().equals(Alliance.Red) ? "red_" : "blue_";
        SequentialCommandGroup command = new SequentialCommandGroup(
            // TODO: ARM DROPOFF
            getPathCommand(swerve, "auto_" + alliance + piece + "_" + goal) 
        );
        if (balance) {
            command.addCommands(
                getPathCommand(swerve, "auto_balance")
            );
        }
        if (piece == PieceCount.TWO) {
            command.addCommands(
                getPathCommand(swerve, "auto_" + alliance + piece + "_" + goal + "_pathA"),
                // TODO: ARM PICKUP
                getPathCommand(swerve, "auto_" + alliance + piece + "_" + goal + "_pathB")
                // TODO: MORE ARM DROPOFF
            );
        }
        return command;
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
