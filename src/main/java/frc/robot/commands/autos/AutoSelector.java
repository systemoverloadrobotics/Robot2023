package frc.robot.commands.autos;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.RotationControlledSwerveDrive;
import frc.robot.commands.SlightlyMoreComplexBalance;
import frc.robot.subsystems.Swerve;

public class AutoSelector {
        
    private final SendableChooser<Command> autoSelector = new SendableChooser<Command>();

    public AutoSelector(Swerve swerve) {

        autoSelector.setDefaultOption("GO_STRAIGHT", new SequentialCommandGroup(
            new RotationControlledSwerveDrive(swerve, () -> 0, () -> -3, () -> 90, () -> false).withTimeout(0.5),
            new RotationControlledSwerveDrive(swerve, () -> 0, () -> 3, () -> 90, () -> false).withTimeout(3),
            new RotationControlledSwerveDrive(swerve, () -> 0, () -> 0, () -> 90, () -> false).withTimeout(20)

        ));
        autoSelector.setDefaultOption("DO_NOTHING2", null);
        // RED
        autoSelector.addOption("AUTO_BALANCE", new SlightlyMoreComplexBalance(swerve));
        SmartDashboard.putData("Autos", autoSelector);
    }

public Command getAuto() {
        return autoSelector.getSelected();
    }
}
