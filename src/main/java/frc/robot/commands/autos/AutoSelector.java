package frc.robot.commands.autos;

import java.sql.Driver;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.autos.AutoPaths.PieceCount;
import frc.robot.commands.autos.AutoPaths.ScoringPosition;
import frc.robot.commands.autos.AutoPaths.StartingPosition;
import frc.robot.subsystems.Swerve;

public class AutoSelector {
    private final SendableChooser<Command> autoSelector = new SendableChooser<Command>();

    public AutoSelector(Swerve swerve) {

        autoSelector.setDefaultOption("DO_NOTHING", null);
        
        // RED
        autoSelector.addOption("AUTO_ONE_PIECE_LEFT", AutoPaths.createAutoCommand(swerve,
                ScoringPosition.HIGH, PieceCount.ONE, false, StartingPosition.LEFT));
        autoSelector.addOption("AUTO_ONE_PIECE_MIDDLE", AutoPaths.createAutoCommand(swerve,
                ScoringPosition.HIGH, PieceCount.ONE, false, StartingPosition.MIDDLE));
        autoSelector.addOption("AUTO_ONE_PIECE_RIGHT", AutoPaths.createAutoCommand(swerve,
                ScoringPosition.HIGH, PieceCount.ONE, false, StartingPosition.RIGHT));
        autoSelector.addOption("AUTO_ONE_PIECE_BALANCE_LEFT", AutoPaths.createAutoCommand(
                swerve, ScoringPosition.HIGH, PieceCount.ONE, true, StartingPosition.LEFT));
        autoSelector.addOption("AUTO_ONE_PIECE_BALANCE_MIDDLE", AutoPaths.createAutoCommand(
                swerve, ScoringPosition.HIGH, PieceCount.ONE, true, StartingPosition.MIDDLE));
        autoSelector.addOption("AUTO_ONE_PIECE_BALANCE_RIGHT", AutoPaths.createAutoCommand(
                swerve, ScoringPosition.HIGH, PieceCount.ONE, true, StartingPosition.RIGHT));
        autoSelector.addOption("AUTO_TWO_PIECE_LEFT", AutoPaths.createAutoCommand(swerve,
                ScoringPosition.HIGH, PieceCount.ONE, false, StartingPosition.LEFT));
        autoSelector.addOption("AUTO_TWO_PIECE_MIDDLE", AutoPaths.createAutoCommand(swerve,
                ScoringPosition.HIGH, PieceCount.ONE, false, StartingPosition.MIDDLE));
        autoSelector.addOption("AUTO_TWO_PIECE_RIGHT", AutoPaths.createAutoCommand(swerve,
                ScoringPosition.HIGH, PieceCount.ONE, false, StartingPosition.RIGHT));
        
        SmartDashboard.putData("Autos", autoSelector);
    }

    public Command getAuto() {
        return autoSelector.getSelected();
    }
}
