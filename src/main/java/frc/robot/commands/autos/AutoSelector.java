package frc.robot.commands.autos;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.MoveStraightAuton;
import frc.robot.commands.SlightlyMoreComplexBalance;
import frc.robot.commands.autos.AutoPaths.PieceCount;
import frc.robot.commands.autos.AutoPaths.ScoringPosition;
import frc.robot.commands.autos.AutoPaths.StartingPosition;
import frc.robot.subsystems.Swerve;

public class AutoSelector {
        
    private final SendableChooser<Command> autoSelector = new SendableChooser<Command>();

    public AutoSelector(Swerve swerve) {
        autoSelector.setDefaultOption("DO_NOTHING", null);
        // RED
        autoSelector.addOption("AUTO_BALANCE", new SlightlyMoreComplexBalance(swerve));
        autoSelector.addOption("GO_STRAIGHT", new MoveStraightAuton(swerve, 1));
        SmartDashboard.putData("Autos", autoSelector);
    }

public Command getAuto() {
        return autoSelector.getSelected();
    }
}
