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
import frc.robot.Robot;
import frc.robot.subsystems.Swerve;

/**
 * Run auto commands
 */
// TODO: enter PID values
public class AutoPaths {
    public static PPSwerveControllerCommand getPathCommand(Swerve swerve, PathName path) {
        PathPlannerTrajectory trajectory = PathPlanner.loadPath(path.toString(),
                Constants.Swerve.SWERVE_MAX_AUTO_SPEED, Constants.Swerve.SWERVE_MAX_ACCELERATION);
        PPSwerveControllerCommand command = new PPSwerveControllerCommand(trajectory,
                swerve::getOdometryPose, Constants.RobotDimensions.SWERVE_DRIVE_KINEMATICS,
                Constants.Auto.X_PID_CONTROLLER, Constants.Auto.Y_PID_CONTROLLER,
                Constants.Auto.ROT_PID_CONTROLLER, swerve::setModuleStates, swerve);
        return command;
    }

    public static Command createAutoCommand(Swerve swerve, ScoringPosition goal, PieceCount piece, boolean balance, StartingPosition startingPos) {
       
        // Do arm stuff here for config from ScoringPosition
        SequentialCommandGroup autoCommandGroup = new SequentialCommandGroup();
        autoCommandGroup.addCommands(); // TODO: arm command for score
        switch (piece) {
            case ONE:
                autoCommandGroup.addCommands(getPathCommand(swerve,getPickupPath(startingPos) ));
                if (balance) {
                    autoCommandGroup.addCommands(getPathCommand(swerve,getBalancePrepPath()));
                    autoCommandGroup.addCommands(); // TODO: Balance commands
                }
                break;
            case TWO:
                autoCommandGroup.addCommands(getPathCommand(swerve, getPickupPath( startingPos)));
                autoCommandGroup.addCommands(); // TODO: arm command for pickup
                autoCommandGroup.addCommands(getPathCommand(swerve, getDropoffPath( startingPos)));
                autoCommandGroup.addCommands(); // TODO: arm command for score
        }
        return autoCommandGroup;
    }

    private static PathName getBalancePrepPath() {
         if(Robot.getAllianceColor().equals(Alliance.Red)){
            return PathName.AUTO_BALANCE_PREP_RED;
         }
         return PathName.AUTO_BALANCE_PREP_BLUE;
    }

    private static PathName getPickupPath( StartingPosition startingPos) {
        switch(startingPos){
            case LEFT:
            if(Robot.getAllianceColor().equals(Alliance.Red)){
                return PathName.AUTO_RED_LEFT_TAXI;
            }
            return PathName.AUTO_BLUE_LEFT_TAXI;
            case MIDDLE:
            if(Robot.getAllianceColor().equals(Alliance.Red)){
                return PathName.AUTO_RED_MIDDLE_TAXI;
            }
            return PathName.AUTO_BLUE_MIDDLE_TAXI;
            case RIGHT:
            if(Robot.getAllianceColor().equals(Alliance.Red)){
                return PathName.AUTO_RED_RIGHT_TAXI;
            }
            return PathName.AUTO_BLUE_RIGHT_TAXI;
        }
        return null;
    }

    private static PathName getDropoffPath(StartingPosition startingPos) {
        switch(startingPos){
            case LEFT:
            if(Robot.getAllianceColor().equals(Alliance.Red)){
                return PathName.AUTO_RED_LEFT_TWO_PIECE;
            }
            return PathName.AUTO_BLUE_LEFT_TWO_PIECE;
            case MIDDLE:
            if(Robot.getAllianceColor().equals(Alliance.Red)){
                return PathName.AUTO_RED_MIDDLE_TWO_PIECE;
            }
            return PathName.AUTO_BLUE_MIDDLE_TWO_PIECE;
            case RIGHT:
            if(Robot.getAllianceColor().equals(Alliance.Red)){
                return PathName.AUTO_RED_RIGHT_TWO_PIECE;
            }
            return PathName.AUTO_BLUE_RIGHT_TWO_PIECE;
        }
        return null;
    }

    public enum StartingPosition {
        LEFT("left"), MIDDLE("middle"), RIGHT("right");

       private String pos;

        StartingPosition(String pos) {
            this.pos = pos;
        }

        @Override
        public String toString() {
            return pos;
        }
    }
    public enum PieceCount {
        ONE("one"), TWO("two");

        private String id;

        PieceCount(String str) {
            id = str;
        }

        @Override
        public String toString() {
            return id;
        }
    }


    public enum PathName {
        AUTO_RED_LEFT_TAXI("auto_red_left_taxi"), 
        AUTO_RED_MIDDLE_TAXI("auto_red_middle_taxi"), 
        AUTO_RED_RIGHT_TAXI("auto_red_right_taxi"), 
        AUTO_RED_LEFT_TWO_PIECE("auto_red_left_two_dropoff"), 
        AUTO_RED_MIDDLE_TWO_PIECE("auto_red_middle_two_dropff"), 
        AUTO_RED_RIGHT_TWO_PIECE("auto_red_right_two_dropoff"),
        AUTO_BLUE_LEFT_TAXI("auto_blue_left_taxi"), 
        AUTO_BLUE_RIGHT_TAXI("auto_blue_middle_taxi"), 
        AUTO_BLUE_MIDDLE_TAXI("auto_blue_right_taxi"), 
        AUTO_BLUE_LEFT_TWO_PIECE("auto_blue_left_two_dropoff"), 
        AUTO_BLUE_MIDDLE_TWO_PIECE("auto_blue_middle_two_dropff"), 
        AUTO_BLUE_RIGHT_TWO_PIECE("auto_blue_right_two_dropoff"),
        AUTO_BALANCE_PREP_BLUE("auto_balance_prep_blue"), 
        AUTO_BALANCE_PREP_RED("auto_balance_prep_red");



        private String path;

        PathName(String path) {
            this.path = path;
        }

        @Override
        public String toString() {
            return path;
        }
    }

    public enum ScoringPosition {
        HIGH("high"), MID("mid");

        private String id;

        ScoringPosition(String str) {
            id = str;
        }

        @Override
        public String toString() {
            return id;
        }
    }
}
