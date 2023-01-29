package frc.robot.autos;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.autos.BaseAuto;
import frc.robot.subsystems.Swerve;

/**
 * Autonomous that moves to a piece and back to score
 */
public class OnePiece extends BaseAuto {
    Swerve swerve;

    
    public OnePiece(Swerve swerve) {
        super(swerve);
        PathPlannerTrajectory Piece = PathPlanner.loadPath("Piece", 3.97, 3);
        PPSwerveControllerCommand firstCommand = baseSwerveCommand(Piece);
        PathPlannerState initialState = Piece.getInitialState();
       

       

    }
}