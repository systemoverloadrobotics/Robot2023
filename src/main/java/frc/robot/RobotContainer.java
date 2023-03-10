// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.FinetuneArm;
import frc.robot.commands.SwerveDrive;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Claw;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.MoveToGrid;
import frc.robot.commands.MoveToHumanPlayer;
import frc.robot.commands.MoveToScoringLocation;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.commands.autos.AutoSelector;
import frc.robot.subsystems.Swerve;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.IntelligentScoring.ScoringLocations;
import frc.robot.subsystems.DriveTrainPoseEstimator;
import frc.robot.subsystems.IntelligentScoring;

import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.Led;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    @SuppressWarnings("unused")
    private final java.util.logging.Logger logger;

    // The robot's subsystems and commands are defined here...
    private final Swerve swerve;
    private final AutoSelector autoSelector;
    private final DriveTrainPoseEstimator poseEstimator;
    private final Vision vision;
    private final IntelligentScoring intelligentScoring;
    private ArmSubsystem arm;
    private Claw claw;
    private Led led;

    //@formatter:off
    private Command pickUpGamePieceLow = new FunctionalCommand(() -> {}, 
            () -> arm.setPosition(ArmSubsystem.ArmHeight.LOW), (a) -> arm.stop(), () -> arm.withinRange(), arm);
    private Command pickUpGamePieceTray = new FunctionalCommand(() -> {},
            () -> arm.setPosition(ArmSubsystem.ArmHeight.TRAY), (a) -> arm.stop(), () -> arm.withinRange(), arm);
    private Command depositGamePieceMid = new FunctionalCommand(() -> {},
            () -> arm.setPosition(ArmSubsystem.ArmHeight.MID), (a) -> arm.stop(), () -> arm.withinRange(), arm);
    private Command depositGamePieceHigh = new FunctionalCommand(() -> {},
            () -> arm.setPosition(ArmSubsystem.ArmHeight.HIGH), (a) -> arm.stop(), () -> arm.withinRange(), arm);
    private Command intakeClaw = new FunctionalCommand(() -> {},
            () -> claw.intake(), (a) -> claw.stop(), () -> false, claw);
    private Command outtakeClaw = new FunctionalCommand(() -> {},
            () -> claw.outtake(), (a) -> claw.stop(), () -> false, claw);
    private Command stowArm = new FunctionalCommand(() -> {},
            () -> arm.setPosition(ArmSubsystem.ArmHeight.STOW), (a) -> arm.stop(), () -> arm.withinRange(), claw);
    //@formatter:on

    private Command finetuneArm = new FinetuneArm(arm, Constants.Input.ARM_MANUAL_MOVEMENT_UP_DOWN.get(),
            Constants.Input.ARM_MANUAL_MOVEMENT_FORWARD_BACKWARD.get());

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        logger = java.util.logging.Logger.getLogger(RobotContainer.class.getName());

        poseEstimator = new DriveTrainPoseEstimator();
        vision = new Vision();
        arm = new ArmSubsystem();
        claw = new Claw();
        led = new Led();
        swerve = new Swerve();
        intelligentScoring = new IntelligentScoring(vision, poseEstimator);
        autoSelector = new AutoSelector(swerve);

        // Configure the button bindings
        configureButtonBindings();

        // Configure arm zero
        configureArm();
    }

    private final Command ledCommandPurple = new RunCommand(() -> {
        led.setLEDColor(Color.kAquamarine);
    }, led);
    private final Command ledCommandYellow = new RunCommand(() -> {
        led.setLEDColor(Color.kYellow);
    }, led);

    /**
     * Use this method to define your button->command mappings. Buttons can be created by instantiating a
     * {@link GenericHID} or one of its subclasses ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}),
     * and then passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        swerve.setDefaultCommand(new SwerveDrive(swerve, () -> -Constants.Input.SWERVE_X_INPUT.get().getAsDouble(),
                () -> -Constants.Input.SWERVE_Y_INPUT.get().getAsDouble(),
                Constants.Input.SWERVE_ROTATION_INPUT.get()));

        arm.setDefaultCommand(finetuneArm);
        Constants.Input.CLAW_IN.get().whileTrue(intakeClaw);
        Constants.Input.CLAW_OUT.get().whileTrue(outtakeClaw);

        Constants.Input.LED_TRIGGER_PURPLE.get().whileTrue(ledCommandPurple);
        Constants.Input.LED_TRIGGER_YELLOW.get().whileTrue(ledCommandYellow);

        // scoring
        Constants.Input.POSITION_TO_CLOSEST_GRID.get()
                .onTrue(new MoveToGrid(poseEstimator, swerve, vision, intelligentScoring));
        Constants.Input.POSITION_TO_HUMAN_PLAYER.get().onTrue(new MoveToHumanPlayer(swerve, poseEstimator));

        Constants.Input.UPPER_LEFT_CONE.get().onTrue(new MoveToScoringLocation(poseEstimator, swerve, vision, arm, claw,
                intelligentScoring, ScoringLocations.UPPER_LEFT_CONE));
        Constants.Input.UPPER_MIDDLE_CUBE.get().onTrue(new MoveToScoringLocation(poseEstimator, swerve, vision, arm,
                claw, intelligentScoring, ScoringLocations.UPPER_MIDDLE_CUBE));
        Constants.Input.UPPER_RIGHT_CONE.get().onTrue(new MoveToScoringLocation(poseEstimator, swerve, vision, arm,
                claw, intelligentScoring, ScoringLocations.UPPER_RIGHT_CONE));
        Constants.Input.MIDDLE_LEFT_CONE.get().onTrue(new MoveToScoringLocation(poseEstimator, swerve, vision, arm,
                claw, intelligentScoring, ScoringLocations.MIDDLE_LEFT_CONE));
        Constants.Input.MIDDLE_MIDDLE_CUBE.get().onTrue(new MoveToScoringLocation(poseEstimator, swerve, vision, arm,
                claw, intelligentScoring, ScoringLocations.MIDDLE_MIDDLE_CUBE));
        Constants.Input.MIDDLE_RIGHT_CONE.get().onTrue(new MoveToScoringLocation(poseEstimator, swerve, vision, arm,
                claw, intelligentScoring, ScoringLocations.MIDDLE_RIGHT_CONE));
        Constants.Input.HYBRID_LEFT.get().onTrue(new MoveToScoringLocation(poseEstimator, swerve, vision, arm, claw,
                intelligentScoring, ScoringLocations.HYBRID_LEFT));
        Constants.Input.HYBRID_MIDDLE.get().onTrue(new MoveToScoringLocation(poseEstimator, swerve, vision, arm, claw,
                intelligentScoring, ScoringLocations.HYBRID_MIDDLE));
        Constants.Input.HYBRID_RIGHT.get().onTrue(new MoveToScoringLocation(poseEstimator, swerve, vision, arm, claw,
                intelligentScoring, ScoringLocations.HYBRID_RIGHT));
    }

    private void configureArm() {
        // TODO: setup arm offset
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return autoSelector.getAuto();
    }
}
