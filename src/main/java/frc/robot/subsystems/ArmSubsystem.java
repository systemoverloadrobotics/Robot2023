// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.sorutil.SorMath;
import frc.sorutil.motor.MotorConfiguration;
import frc.sorutil.motor.SensorConfiguration;
import frc.sorutil.motor.SuSparkMax;
import frc.sorutil.motor.SuTalonFx;
import frc.sorutil.motor.SuController.ControlMode;

public class ArmSubsystem extends SubsystemBase {

    private final java.util.logging.Logger logger;
    private final Logger aLogger;

    private SuTalonFx jointA;
    private SuTalonFx jointB;
    private SuSparkMax cascade;
    private DutyCycleEncoder jointAbsoluteEncoder;
    private Pair<Double, Double> intendedPosition;
    private boolean safeMode;
    private boolean retractCascade;
    private boolean preventExtension;
    private final Timer timerArmAnglePosition = new Timer();
    private final LinearFilter filter;

    private boolean flag;

    private static class ArmModel {
        private final Mechanism2d mech = new Mechanism2d(2, 2);
        private final MechanismLigament2d elevatorExtension;
        private final MechanismLigament2d armBack;

        public ArmModel(String name) {
            var root = mech.getRoot(name, 1, 0);
            var upright = root.append(new MechanismLigament2d("upright", Constants.Arm.ARM_PIVOT_Y, 90, 4,
                    new Color8Bit(Color.kDarkKhaki)));
            armBack = upright.append(new MechanismLigament2d("armBack", Constants.Arm.MIN_ARM_LENGTH, 90, 8,
                    new Color8Bit(Color.kAquamarine)));
            armBack.append(new MechanismLigament2d("arm", Constants.Arm.MIN_ARM_LENGTH * 2, 180, 8,
                    new Color8Bit(Color.kAquamarine)));
            elevatorExtension = armBack
                    .append(new MechanismLigament2d("elevatorExtension", 0, 180, 4, new Color8Bit(Color.kAquamarine)));
            elevatorExtension.append(new MechanismLigament2d("elevator", Constants.Arm.MIN_ARM_LENGTH * 2, 0, 4,
                    new Color8Bit(Color.kGreen)));
        }

        /**
         * update changes the position of the arm model to match the arguments passed.
         * 
         * The first argument should be angle of the arm in degrees measured from straight out as 0, and the second
         * argument should be the length of the arm from the pivot, accounting for the minimum length of the arm.
         */
        public void update(double angle, double length) {
            armBack.setAngle(angle);
            elevatorExtension.setLength(length);
        }

        public Mechanism2d asMechanism() {
            return mech;
        }
    }

    // Logging visualization
    private ArmModel intentMechanism = new ArmModel("ArmIntent");
    private ArmModel currentMechanism = new ArmModel("ArmCurrent");

    private final TrapezoidProfile.Constraints constraintsAngle = new TrapezoidProfile.Constraints(720, 360); // Units/s,

    private TrapezoidProfile.State goalAngle;
    private TrapezoidProfile.State currentAngle;

    private final TrapezoidProfile.Constraints constraintsArmLength = new TrapezoidProfile.Constraints(2, 4); // Units/s,
                                                                                                                 // Units/s^2
    private TrapezoidProfile.State goalArmLength;
    private TrapezoidProfile.State currentArmLength;

    /** Creates a new ArmSubsystem. */
    public ArmSubsystem() {
        logger = java.util.logging.Logger.getLogger(ArmSubsystem.class.getName());
        aLogger = Logger.getInstance();

        MotorConfiguration jointMotorConfig = new MotorConfiguration();

        jointMotorConfig.setPidProfile(Constants.Arm.ARM_PID_PROFILE);
        jointMotorConfig.setCurrentLimit(Constants.Arm.ARM_JOINT_CURRENT_LIMIT);
        SensorConfiguration jointSensorConfiguration =
                new SensorConfiguration(new SensorConfiguration.IntegratedSensorSource(83.2));
        jointMotorConfig.setMaxOutput(0.1);
        jointA = new SuTalonFx(new WPI_TalonFX(Constants.Motor.ARM_JOINT_INDEX), "Joint Motor A", jointMotorConfig,
                jointSensorConfiguration);
        jointB = new SuTalonFx(new WPI_TalonFX(Constants.Motor.ARM_JOINT_FOLLOWER_INDEX), "Joint Motor B",
                jointMotorConfig, jointSensorConfiguration);
        jointB.follow(jointA);
        ((WPI_TalonFX) jointA.rawController()).configClosedloopRamp(0.25);
        ((WPI_TalonFX) jointB.rawController()).configClosedloopRamp(0.25);

        MotorConfiguration cascadeMotorConfig = new MotorConfiguration();
        cascadeMotorConfig.setPidProfile(Constants.Arm.CASCADE_PID_PROFILE);
        cascadeMotorConfig.setCurrentLimit(Constants.Arm.ARM_CASCADE_CURRENT_LIMIT);
        cascadeMotorConfig.setMaxOutput(0.4);
        SensorConfiguration cascadeSensorConfiguration =
                new SensorConfiguration(new SensorConfiguration.IntegratedSensorSource(9));

        cascade = new SuSparkMax(new CANSparkMax(Constants.Motor.ARM_CASCADE_INDEX, MotorType.kBrushless),
                "Cascade Motor", cascadeMotorConfig, cascadeSensorConfiguration);
        jointAbsoluteEncoder = new DutyCycleEncoder(Constants.Arm.ARM_ABSOLUTE_ENCODER_PORT);
        // Absolute Encoder is 8192 / rot
        timerArmAnglePosition.start();

        resetArmProfile();
        setPosition(getManipulatorPositionRTheta());
        
        currentAngle = new TrapezoidProfile.State(getDegreesJoint(), jointA.outputVelocity());
        currentArmLength = new TrapezoidProfile.State(cascade.outputPosition(), cascade.outputVelocity());
        filter = LinearFilter.movingAverage(40);

        logger.info("Arm Initialized.");
    }

    /*
     * Reference plane for 2d coordinate has origin at joint with plane parallel to side view, r and theta are first and second
     */
    public void setPosition(ArmSubsystem.ArmHeight height) {
        setPosition(height.getCoordinates());
    }

    /*
     * Reference plane for 2d coordinate has origin at joint with plane parallel to side view, r and theta are first and second
     */
    public void setPosition(Pair<Double, Double> pair) {
        aLogger.recordOutput("Arm/IntendedR", pair.getFirst());
        aLogger.recordOutput("Arm/IntendedTheta", pair.getSecond());
        intendedPosition = pair;
        goalAngle = new TrapezoidProfile.State(pair.getFirst(), 0);
        goalArmLength =
                new TrapezoidProfile.State(pair.getSecond(), 0);
    }

    public Pair<Double, Double> getIntendedPosition() {
        return intendedPosition;
    }

    public void stop() {
        setPosition(getManipulatorPositionRTheta());
    }

    public Pair<Double, Double> getManipulatorPositionRTheta() {
        double r = cascadeDegreesToFeet(cascade.outputPosition());
        double theta = getDegreesJoint();
        Pair<Double, Double> position = new Pair<>(theta, r);
        return position;
    }

    public Pair<Double, Double> getManipulatorPositionXY() {
        var cascadeMult = cascadeDegreesToFeet(cascade.outputPosition()) + (17/12);
        var pose = new Translation2d(Math.cos(Math.toRadians(getDegreesJoint() - 90)) * cascadeMult, Math.sin(Math.toRadians(getDegreesJoint() - 90)) * cascadeMult);
        aLogger.recordOutput("armX", pose.getX());
        aLogger.recordOutput("armY", pose.getY());
        return new Pair<Double, Double>(pose.getX(), pose.getY());
    }

    private double cascadeDegreesToFeet(double degrees) {
        return Constants.Arm.ARM_CASCADE_STARTING_HEIGHT + (degrees / Constants.Arm.ARM_CASCADE_DEG_PER_FOOT);
    }

    private double cascadeFeetToDegrees(double feet) {
        return (feet - Constants.Arm.ARM_CASCADE_STARTING_HEIGHT) * Constants.Arm.ARM_CASCADE_DEG_PER_FOOT;
    }

    // extension - inches
    // angle - degrees
    public double calcFeedForwardJoint(double angle, double extension) {
        double sinAngle = Math.cos(Math.toRadians(angle));
        double torque = (-1.72 * 16.53 * sinAngle) + (3.19 * extension * sinAngle) + (9.33 * 44.02 * sinAngle)
                + (5.33 * (extension + 19.53) * sinAngle);
        return (torque / 290);// * 0.35d;
    }

    public double calcFeedForwardCascade(double angle) {
        return Constants.Arm.ARM_CASCADE_MAX_FEEDFORWARD * -Math.cos(angle);
    }

    private double getDegreesJoint() {
      return jointA.outputPosition();
        // return ((WPI_TalonFX) (jointA.rawController())).getSelectedSensorPosition() / 78.54 / 2048 * 360;
        // return SorMath.ticksToDegrees(((WPI_TalonFX) jointA.rawController()).getSelectedSensorPosition(), 2048) / 95;
    }

    @Override
    public void periodic() {
        aLogger.recordOutput("Arm/IntendedR", intendedPosition.getSecond());
        aLogger.recordOutput("Arm/IntendedTheta", intendedPosition.getFirst());
        aLogger.recordOutput("Arm/IntendedPosition", intentMechanism.asMechanism());
        aLogger.recordOutput("Arm/CurrentPosition", currentMechanism.asMechanism());
        getManipulatorPositionXY();

        if (timerArmAnglePosition.hasElapsed(4)) {
            jointA.setSensorPosition(Constants.Arm.ARM_JOINT_OFFSET - jointAbsoluteEncoder.getAbsolutePosition());
            jointB.setSensorPosition(Constants.Arm.ARM_JOINT_OFFSET - jointAbsoluteEncoder.getAbsolutePosition());
            timerArmAnglePosition.reset();
            timerArmAnglePosition.stop();
        }
        if (!flag) {
            cascade.set(ControlMode.VOLTAGE, Constants.Arm.ARM_ZEROING_VOLTAGE);
            var current = filter.calculate(((CANSparkMax) cascade.rawController()).getOutputCurrent());
            if (current > 15) {
                cascade.set(ControlMode.VOLTAGE, 0);
                cascade.setSensorPosition(0);
                flag = true;
            }
            currentAngle = new TrapezoidProfile.State(jointAbsoluteEncoder.get(), jointA.outputVelocity() * 6);
            var profileAngle = new TrapezoidProfile(constraintsAngle, goalAngle, armAngleSetpoint);
            armAngleSetpoint = profileAngle.calculate(Constants.ROBOT_PERIOD);
            jointA.set(ControlMode.POSITION, armAngleSetpoint.position, calcFeedForwardJoint(jointAbsoluteEncoder.get(), cascadeDegreesToFeet(cascade.outputPosition())));
            aLogger.recordOutput("Arm/NeededAngle", armAngleSetpoint.position);
            return;
        }
        // This method will be called once per scheduler run
        // When the arm is detected to be in the forbidden zone, the variable state for pause and preventExtension
        // typically
        // goes:
        // safeMode/preventExtension = true -> pause = false (when arm is fully retracted) -> preventExtension = false
        // (when arm
        // is
        // moved to right position)
        refreshArmGoal();

        currentMechanism.update(getDegreesJoint(), getManipulatorPositionRTheta().getFirst());

        aLogger.recordOutput("Arm/TestingPositionSecond", intendedPosition.getSecond());
        aLogger.recordOutput("Arm/TestingPositionManipulatorSecond", getManipulatorPositionXY().getSecond());

        aLogger.recordOutput("Arm/CurrentAngle", getDegreesJoint());
        aLogger.recordOutput("Arm/CurrentAbsAngle", jointAbsoluteEncoder.getAbsolutePosition());
        // outputVelocity is in RPM, we want it in degrees/sec
        aLogger.recordOutput("Arm/CurrentAngleVelocity", jointA.outputVelocity() * 6.0);
        aLogger.recordOutput("Arm/CurrentCascade", cascadeDegreesToFeet(cascade.outputPosition()));
        aLogger.recordOutput("Arm/TestAngle", goalAngle.position);
        aLogger.recordOutput("Arm/GoalCasc", goalArmLength.position);
    }

    private void refreshArmGoal() {
        aLogger.recordOutput("Arm/Safe", safeMode);
        if (futureArmSafetyPrediction() && !safeMode) { // && !safeMode
            setUpdatedArmState();
        } else if (!futureArmSafetyPrediction() || safeMode) { //  && !safeMode
            if (!safeMode) {
                jointA.set(ControlMode.POSITION, jointA.outputPosition(), calcFeedForwardJoint(jointA.outputPosition(), cascadeDegreesToFeet(cascade.outputPosition())));
                safeMode = true;
                retractCascade = true;
                preventExtension = true;
            }

            if (retractCascade) {
                cascade.set(ControlMode.POSITION, 16, calcFeedForwardCascade(jointA.outputPosition()));
                if (between(cascade.outputPosition(), 0, 32)) {
                    retractCascade = false;
                }
            }
            else if (preventExtension) {
                jointA.set(ControlMode.POSITION, intendedPosition.getFirst(), calcFeedForwardJoint(jointA.outputPosition(), cascadeDegreesToFeet(cascade.outputPosition())));
                cascade.set(ControlMode.POSITION, 16, calcFeedForwardCascade(jointA.outputPosition()));
                if (intendedPosition.getFirst() > Constants.Arm.ARM_MAX_ANGLE_COLLISION_A) {
                    safeMode = false;
                    preventExtension = false;
                }
            }
        }
    }

    private TrapezoidProfile.State armAngleSetpoint = new TrapezoidProfile.State();
    private TrapezoidProfile.State armCascadeSetpoint = new TrapezoidProfile.State();

    public void resetArmProfile() {
        intendedPosition = getManipulatorPositionRTheta();
        armAngleSetpoint = new TrapezoidProfile.State(getDegreesJoint(), 0);
        goalAngle = new TrapezoidProfile.State(getDegreesJoint(), 0);
        flag = false;
        goalArmLength = new TrapezoidProfile.State(0, 0);
        armCascadeSetpoint = new TrapezoidProfile.State(0, 0);

        safeMode = false;
        retractCascade = false;
        preventExtension = false;
      }

    private void setUpdatedArmState() {
        //aLogger.recordOutput("Arm/", null);
        var profileAngle = new TrapezoidProfile(constraintsAngle, goalAngle, armAngleSetpoint);
        armAngleSetpoint = profileAngle.calculate(Constants.ROBOT_PERIOD);

        var profileArmLength = new TrapezoidProfile(constraintsArmLength, goalArmLength, armCascadeSetpoint);
        aLogger.recordOutput("Arm/TestAngle", goalAngle.position);
        armCascadeSetpoint = profileArmLength.calculate(Constants.ROBOT_PERIOD);

        cascade.set(ControlMode.POSITION, cascadeFeetToDegrees(armCascadeSetpoint.position), calcFeedForwardCascade(jointAbsoluteEncoder.get()));
        // Above means going to 0, needs negative
        jointA.set(ControlMode.POSITION, armAngleSetpoint.position, calcFeedForwardJoint(jointAbsoluteEncoder.get(), cascadeDegreesToFeet(cascade.outputPosition())));
        
        aLogger.recordOutput("Arm/NeededAngle", armAngleSetpoint.position);
        aLogger.recordOutput("Arm/NeededCascade", armCascadeSetpoint.position);
        aLogger.recordOutput("Arm/OutputVelocity", jointA.outputVelocity() * 6);
    }

    private boolean futureArmSafetyPrediction() {
        // 1/4 seconds should give us enough time to respond
        double estimatedLength = cascadeDegreesToFeet(
                cascade.outputPosition() + (cascade.outputVelocity() * Constants.Arm.ARM_PREDICTIVE_TIMESPAN * 6)); 
        double estimatedAngle =
                (getDegreesJoint() + (jointA.outputVelocity() * Constants.Arm.ARM_PREDICTIVE_TIMESPAN * 6)) % 360;
        var thet = getManipulatorPositionXY();

        return isSafeFromGroundCollision(thet.getSecond()) && isAngleSafe(estimatedAngle);
    }


    private boolean isSafeFromGroundCollision(double length) {
        return length > Constants.Arm.ARM_HEIGHT_FROM_BASE;
    }

    private boolean isAngleSafe(double estimatedAngle) {
        aLogger.recordOutput("Arm/PredictedAngle", estimatedAngle);
        return !between(estimatedAngle, Constants.Arm.ARM_MIN_ANGLE_COLLISION_A, Constants.Arm.ARM_MAX_ANGLE_COLLISION_A);
    }

    private boolean isArmRetracted() {
        return cascade.outputPosition() < Constants.Arm.ARM_CASCADE_TOLERANCE;
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    private boolean between(double in, double low, double high) {
        return in > low && in < high;
    }

    public boolean withinRange() {
        return between(getManipulatorPositionRTheta().getFirst(),
                intendedPosition.getFirst() - Constants.Arm.ARM_POSITION_TOLERANCE,
                intendedPosition.getFirst() + Constants.Arm.ARM_POSITION_TOLERANCE)
                && between(getManipulatorPositionRTheta().getSecond(),
                        intendedPosition.getSecond() - Constants.Arm.ARM_POSITION_TOLERANCE,
                        intendedPosition.getSecond() + Constants.Arm.ARM_POSITION_TOLERANCE);
    }

    public enum ArmHeight {
    //@formatter:off
    LOW(new Pair<Double, Double>(Constants.Arm.ARM_PRESET_LOW_ANGLE,Constants.Arm.ARM_PRESET_LOW_LENGTH)), 
    MID_CUBE(new Pair<Double, Double>(Constants.Arm.ARM_PRESET_MID_CUBE_ANGLE,Constants.Arm.ARM_PRESET_MID_CUBE_LENGTH)), 
    HIGH_CUBE(new Pair<Double, Double>(Constants.Arm.ARM_PRESET_HIGH_CUBE_ANGLE,Constants.Arm.ARM_PRESET_HIGH_CUBE_LENGTH)),
    MID_CONE(new Pair<Double, Double>(Constants.Arm.ARM_PRESET_MID_CONE_ANGLE,Constants.Arm.ARM_PRESET_MID_CONE_LENGTH)), 
    HIGH_CONE(new Pair<Double, Double>(Constants.Arm.ARM_PRESET_HIGH_CONE_ANGLE,Constants.Arm.ARM_PRESET_HIGH_CONE_LENGTH)), 
    TRAY(new Pair<Double, Double>(Constants.Arm.ARM_PRESET_TRAY_ANGLE,Constants.Arm.ARM_PRESET_TRAY_LENGTH)), 
    STOW(new Pair<Double, Double>(Constants.Arm.ARM_PRESET_STOW_ANGLE,Constants.Arm.ARM_PRESET_STOW_LENGTH)),
    TESTA(new Pair<Double, Double>(25.0, 1.0)),
    TESTB(new Pair<Double, Double>(90.0, 1.0));
    //@formatter:on

        private final Pair<Double, Double> coordinates;

        ArmHeight(Pair<Double, Double> coordinates) {
            this.coordinates = coordinates;
        }

        public Pair<Double, Double> getCoordinates() {
            return coordinates;
        }
    }
}
