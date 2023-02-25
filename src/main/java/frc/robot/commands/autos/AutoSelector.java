package frc.robot.commands.autos;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.autos.AutoPaths.PieceCount;
import frc.robot.commands.autos.AutoPaths.ScoringPosition;
import frc.robot.subsystems.Swerve;

public class AutoSelector {
    private final SendableChooser<Command> autoSelector = new SendableChooser<Command>();

    public AutoSelector(Swerve swerve) {

        autoSelector.setDefaultOption("DO_NOTHING",null);
        autoSelector.addOption("AUTO_BLUE_ONE_PIECE_LEFT", AutoPaths.createAutoCommand(swerve,ScoringPosition.HIGH , PieceCount.ONE, false, "left"));
        autoSelector.addOption("AUTO_BLUE_ONE_PIECE_MIDDLE", AutoPaths.createAutoCommand(swerve,ScoringPosition.HIGH , PieceCount.ONE, false, "middle"));
        autoSelector.addOption("AUTO_BLUE_ONE_PIECE_RIGHT", AutoPaths.createAutoCommand(swerve,ScoringPosition.HIGH , PieceCount.ONE, false, "right"));
        autoSelector.addOption("AUTO_BLUE_ONE_PIECE_BALANCE_LEFT", AutoPaths.createAutoCommand(swerve,ScoringPosition.HIGH , PieceCount.ONE, true,"left"));
        autoSelector.addOption("AUTO_BLUE_ONE_PIECE_BALANCE_MIDDLE", AutoPaths.createAutoCommand(swerve,ScoringPosition.HIGH , PieceCount.ONE, true, "middle"));
        autoSelector.addOption("AUTO_BLUE_ONE_PIECE_BALANCE_RIGHT", AutoPaths.createAutoCommand(swerve,ScoringPosition.HIGH , PieceCount.ONE, true, "right"));
        autoSelector.addOption("AUTO_BLUE_TWO_PIECE_LEFT", AutoPaths.createAutoCommand(swerve,ScoringPosition.HIGH , PieceCount.ONE, false, "left"));
        autoSelector.addOption("AUTO_BLUE_TWO_PIECE_MIDDLE", AutoPaths.createAutoCommand(swerve,ScoringPosition.HIGH , PieceCount.ONE, false, "middle"));
        autoSelector.addOption("AUTO_BLUE_TWO_PIECE_RIGHT", AutoPaths.createAutoCommand(swerve,ScoringPosition.HIGH , PieceCount.ONE, false, "right"));

        // RED

        autoSelector.addOption("AUTO_RED_ONE_PIECE_LEFT", AutoPaths.createAutoCommand(swerve,ScoringPosition.HIGH , PieceCount.ONE, false, "left"));
        autoSelector.addOption("AUTO_RED_ONE_PIECE_MIDDLE", AutoPaths.createAutoCommand(swerve,ScoringPosition.HIGH , PieceCount.ONE, false, "middle"));
        autoSelector.addOption("AUTO_RED_ONE_PIECE_RIGHT", AutoPaths.createAutoCommand(swerve,ScoringPosition.HIGH , PieceCount.ONE, false, "right"));
        autoSelector.addOption("AUTO_RED_ONE_PIECE_BALANCE_LEFT", AutoPaths.createAutoCommand(swerve,ScoringPosition.HIGH , PieceCount.ONE, true, "left"));
        autoSelector.addOption("AUTO_RED_ONE_PIECE_BALANCE_MIDDLE", AutoPaths.createAutoCommand(swerve,ScoringPosition.HIGH , PieceCount.ONE, true, "middle"));
        autoSelector.addOption("AUTO_RED_ONE_PIECE_BALANCE_RIGHT", AutoPaths.createAutoCommand(swerve,ScoringPosition.HIGH , PieceCount.ONE, true, "right"));
        autoSelector.addOption("AUTO_RED_TWO_PIECE_LEFT", AutoPaths.createAutoCommand(swerve,ScoringPosition.HIGH , PieceCount.ONE, false, "left"));
        autoSelector.addOption("AUTO_RED_TWO_PIECE_MIDDLE", AutoPaths.createAutoCommand(swerve,ScoringPosition.HIGH , PieceCount.ONE, false, "middle"));
        autoSelector.addOption("AUTO_RED_TWO_PIECE_RIGHT", AutoPaths.createAutoCommand(swerve,ScoringPosition.HIGH , PieceCount.ONE, false, "right"));



        SmartDashboard.putData("Autos", autoSelector);


    }

    public Command getAuto() {
        return autoSelector.getSelected();
    }
}
