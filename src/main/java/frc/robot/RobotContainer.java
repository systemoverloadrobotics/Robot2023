// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
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
import frc.robot.commands.RotationControlledSwerveDrive;
import frc.robot.commands.SimpleBalance;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.commands.autos.AutoSelector;
import frc.robot.subsystems.Swerve;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.IntelligentScoring.ScoringLocations;
import frc.sorutil.SorMath;
import frc.robot.subsystems.DriveTrainPoseEstimator;
import frc.robot.subsystems.IntelligentScoring;

import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.Led;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and button mappings) should be declared here.
 */
@SuppressWarnings("unused")
public class RobotContainer {
    private final java.util.logging.Logger logger;

    // The robot's subsystems and commands are defined here...
    private final Swerve swerve;
    private final AutoSelector autoSelector;
   // private final DriveTrainPoseEstimator poseEstimator;
    //private final Vision vision;
  //  private final IntelligentScoring intelligentScoring;
    private ArmSubsystem arm;
    private Claw claw;
    private Led led;

    // ---------- Begin Simple Commands ----------

    //@formatter:off
    private final Command moveArmLow;
    private final Command moveArmTray;
    private final Command moveArmMidCube;
    private final Command moveArmHighCube;
    private final Command moveArmMidCone;
    private final Command moveArmHighCone;
    private final Command intakeClawCone;
    private final Command intakeClawCube;
    private final Command outtakeClawMid;
    private final Command outtakeClawHigh;
    private final Command stowArm;
    private final Command resetArmPosition;
    private final Command armTestA;
    private final Command armTestB;
    private final Command lockSwerve;
    private final Command chargeStation;
    
    //@formatter:on

    private Command finetuneArm;
    private final Command ledCommandPurple;
    private final Command ledCommandYellow;

    private final Command driveFacingAlliance;
    private final Command driveFacingHumanPlayer;

    // ---------- End Simple Commands ----------

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        logger = java.util.logging.Logger.getLogger(RobotContainer.class.getName());

       // vision = new Vision();
        arm = new ArmSubsystem();
        claw = new Claw();
        led = new Led();
        swerve = new Swerve();
      //  poseEstimator = new DriveTrainPoseEstimator(swerve, vision);
      //  intelligentScoring = new IntelligentScoring(vision, poseEstimator);
        autoSelector = new AutoSelector(swerve);

        moveArmLow = new FunctionalCommand(() -> {}, 
        () -> arm.setPosition(ArmSubsystem.ArmHeight.LOW), (a) -> {arm.stop();}, () -> false, arm);
        moveArmTray = new FunctionalCommand(() -> {},
                () -> arm.setPosition(ArmSubsystem.ArmHeight.TRAY), (a) -> {arm.stop();}, () -> false, arm);
        moveArmMidCube = new FunctionalCommand(() -> {},
                () -> arm.setPosition(ArmSubsystem.ArmHeight.MID_CUBE), (a) -> {arm.stop();}, () -> false, arm);
        moveArmHighCube = new FunctionalCommand(() -> {},
                () -> arm.setPosition(ArmSubsystem.ArmHeight.HIGH_CUBE), (a) -> {arm.stop();}, () -> false, arm);
        moveArmMidCone = new FunctionalCommand(() -> {},
                () -> arm.setPosition(ArmSubsystem.ArmHeight.MID_CONE), (a) -> {arm.stop();}, () -> false, arm);
        moveArmHighCone = new FunctionalCommand(() -> {},
                () -> arm.setPosition(ArmSubsystem.ArmHeight.HIGH_CONE), (a) -> {arm.stop();}, () -> false, arm);
        intakeClawCone = new FunctionalCommand(() -> {},
                () -> claw.intake(Constants.Claw.CLAW_VELOCITY_IN_CONE), (a) -> {
                        claw.defaultClaw();
                }, () -> false, claw);
        intakeClawCube = new FunctionalCommand(() -> {},
                () -> claw.intake(Constants.Claw.CLAW_VELOCITY_IN_CUBE), (a) -> {
                        claw.stop();
                }, () -> false, claw);
        outtakeClawMid = new FunctionalCommand(() -> {},
                () -> claw.outtake(Constants.Claw.CLAW_VELOCITY_OUT_MID), (a) -> {
                        if (Constants.Input.CLAW_OUT_MID.get().getAsBoolean()) {
                                claw.stop();
                        }
                }, () -> false, claw);
        outtakeClawHigh = new FunctionalCommand(() -> {},
                () -> claw.outtake(Constants.Claw.CLAW_VELOCITY_OUT_HIGH), (a) -> {
                        if (Constants.Input.CLAW_OUT_HIGH.get().getAsBoolean()) {
                                claw.stop();
                        }
                }, () -> false, claw);
        stowArm = new FunctionalCommand(() -> {},
                () -> arm.setPosition(ArmSubsystem.ArmHeight.STOW), (a) -> arm.stop(), () -> false, claw);
        armTestA = new FunctionalCommand(() -> {},
                () -> arm.setPosition(ArmSubsystem.ArmHeight.TESTA), (a) -> arm.stop(), () -> false, claw);
        armTestB = new FunctionalCommand(() -> {},
                () -> arm.setPosition(ArmSubsystem.ArmHeight.TESTB), (a) -> arm.stop(), () -> false, claw);
        lockSwerve = new FunctionalCommand(() -> {}, 
                () -> swerve.lock(), (a) -> swerve.reset(), () -> false, swerve);
        chargeStation = new SimpleBalance(swerve);

        resetArmPosition = new InstantCommand(() -> arm.resetArmProfile(), arm);
        

        finetuneArm = new FinetuneArm(arm, Constants.Input.ARM_MANUAL_MOVEMENT_UP_DOWN.get(),
                Constants.Input.ARM_MANUAL_MOVEMENT_FORWARD_BACKWARD.get());
        
        ledCommandYellow = new RunCommand(() -> {
            led.setLEDColor(Color.kYellow);
        }, led);
        ledCommandPurple = new RunCommand(() -> {
            led.setLEDColor(Color.kPurple);
        }, led);
        

        driveFacingAlliance = new RotationControlledSwerveDrive(swerve, () -> -Constants.Input.SWERVE_X_INPUT.get().getAsDouble(),
                () -> -Constants.Input.SWERVE_Y_INPUT.get().getAsDouble(), () -> 270);
        driveFacingHumanPlayer = new RotationControlledSwerveDrive(swerve, () -> -Constants.Input.SWERVE_X_INPUT.get().getAsDouble(),
                () -> -Constants.Input.SWERVE_Y_INPUT.get().getAsDouble(), () -> 90);

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by instantiating a
     * {@link GenericHID} or one of its subclasses ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}),
     * and then passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        swerve.setDefaultCommand(new SwerveDrive(swerve, () -> -Constants.Input.SWERVE_X_INPUT.get().getAsDouble(),
                () -> -Constants.Input.SWERVE_Y_INPUT.get().getAsDouble(),
                Constants.Input.SWERVE_ROTATION_INPUT.get()));
        Constants.Input.SWERVE_FACE_ALLIANCE.get().whileTrue(driveFacingAlliance);
        Constants.Input.SWERVE_FACE_HUMAN_PLAYER.get().whileTrue(driveFacingHumanPlayer);

        
        Constants.Input.MID_CONE_SCORE.get().toggleOnTrue(moveArmMidCone);
        Constants.Input.MID_CUBE_SCORE.get().toggleOnTrue(moveArmMidCube);
        Constants.Input.HIGH_CONE_SCORE.get().toggleOnTrue(moveArmHighCone);
        Constants.Input.HIGH_CUBE_SCORE.get().toggleOnTrue(moveArmHighCube);
        Constants.Input.LOW_SCORE.get().toggleOnTrue(moveArmLow);
        Constants.Input.TRAY.get().toggleOnTrue(moveArmTray);
        Constants.Input.STOW.get().toggleOnTrue(stowArm);
        Constants.Input.CLAW_IN_CONE.get().whileTrue(intakeClawCone);
        Constants.Input.CLAW_IN_CUBE.get().whileTrue(intakeClawCube);
        Constants.Input.CLAW_OUT_MID.get().toggleOnTrue(outtakeClawMid);
        Constants.Input.CLAW_OUT_HIGH.get().toggleOnTrue(outtakeClawHigh);
        Constants.Input.TEST_A.get().toggleOnTrue(armTestA);
        Constants.Input.TEST_B.get().toggleOnTrue(armTestB);
        Constants.Input.LOCK.get().toggleOnTrue(lockSwerve);

        // Constants.Input.LED_TRIGGER_PURPLE.get().whileTrue(ledCommandPurple);
        // Constants.Input.LED_TRIGGER_YELLOW.get().whileTrue(ledCommandYellow);

        // scoring
        // Constants.Input.POSITION_TO_CLOSEST_GRID.get()
        //         .onTrue(new MoveToGrid(poseEstimator, swerve, intelligentScoring));

        //Constants.Input.POSITION_TO_HUMAN_PLAYER.get().onTrue(new MoveToHumanPlayer(swerve, poseEstimator));

        // Constants.Input.UPPER_LEFT_CONE.get().onTrue(createMoveCommand(ScoringLocations.UPPER_LEFT_CONE));
        // Constants.Input.UPPER_MIDDLE_CUBE.get().onTrue(createMoveCommand(ScoringLocations.UPPER_MIDDLE_CUBE));
        // Constants.Input.UPPER_RIGHT_CONE.get().onTrue(createMoveCommand(ScoringLocations.UPPER_RIGHT_CONE));
        // Constants.Input.MIDDLE_LEFT_CONE.get().onTrue(createMoveCommand(ScoringLocations.MIDDLE_LEFT_CONE));
        // Constants.Input.MIDDLE_MIDDLE_CUBE.get().onTrue(createMoveCommand(ScoringLocations.MIDDLE_MIDDLE_CUBE));
        // Constants.Input.MIDDLE_RIGHT_CONE.get().onTrue(createMoveCommand(ScoringLocations.MIDDLE_RIGHT_CONE));
        // Constants.Input.HYBRID_LEFT.get().onTrue(createMoveCommand(ScoringLocations.HYBRID_LEFT));
        // Constants.Input.HYBRID_MIDDLE.get().onTrue(createMoveCommand(ScoringLocations.HYBRID_MIDDLE));
        // Constants.Input.HYBRID_RIGHT.get().onTrue(createMoveCommand(ScoringLocations.HYBRID_RIGHT));
    }

//     private Command createMoveCommand(ScoringLocations location) {
//         return new MoveToScoringLocation(poseEstimator, swerve, intelligentScoring, location);
//     }

    public void autonomousInit() {
        resetArmPosition.schedule();
    }

    public void teleopInit() {
        resetArmPosition.schedule();
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return autoSelector.getAuto();
        // return chargeStation;
    }

    public void disabledPeriodic() {
        arm.resetArmProfile();
    }
}
