// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.sorutil.SorMath;
import frc.sorutil.motor.MotorConfiguration;
import frc.sorutil.motor.SensorConfiguration;
import frc.sorutil.motor.SuController.ControlMode;
import frc.sorutil.motor.SuSparkMax;
import frc.sorutil.motor.SuTalonFx;

public class ArmSubsystem extends SubsystemBase {
  private final java.util.logging.Logger logger;
  private final Logger aLogger;

  private SuTalonFx jointA;
  private SuTalonFx jointB;
  private SuSparkMax cascade;
  private Pair<Double, Double> intendedPosition;
  private boolean safeMode;
  private boolean retractCascade;
  private boolean preventExtension;

  private static class ArmModel {
    private final Mechanism2d mech = new Mechanism2d(2, 2);
    private final MechanismLigament2d elevatorExtension;
    private final MechanismLigament2d armBack;

    public ArmModel(String name) {
      var root = mech.getRoot(name, 1, 0);
      var upright = root.append(new MechanismLigament2d("upright", Constants.Arm.ARM_PIVOT_Y, 90, 4,
          new Color8Bit(Color.kDarkKhaki)));
      armBack = upright.append(new MechanismLigament2d("armBack", Constants.Arm.MIN_ARM_LENGTH, 90,
          8, new Color8Bit(Color.kAquamarine)));
      armBack.append(new MechanismLigament2d("arm", Constants.Arm.MIN_ARM_LENGTH * 2, 180, 8,
          new Color8Bit(Color.kAquamarine)));
      elevatorExtension = armBack.append(new MechanismLigament2d("elevatorExtension", 0, 180, 4,
          new Color8Bit(Color.kAquamarine)));
      elevatorExtension.append(new MechanismLigament2d("elevator", Constants.Arm.MIN_ARM_LENGTH * 2,
          0, 4, new Color8Bit(Color.kGreen)));
    }

    /**
     * update changes the position of the arm model to match the arguments passed.
     * 
     * The first argument should be angle of the arm in degrees measured from straight out as 0, and the second argument
     * should be the length of the arm from the pivot, accounting for the minimum length of the arm.
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

    private final TrapezoidProfile.Constraints constraintsAngle = new TrapezoidProfile.Constraints(175, 75);  // Units/s, Units/s^2
    private TrapezoidProfile.State goalAngle;
    private TrapezoidProfile.State currentAngle;

    private final TrapezoidProfile.Constraints constraintsArmLength = new TrapezoidProfile.Constraints(175, 75);  // Units/s, Units/s^2
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
        new SensorConfiguration(new SensorConfiguration.IntegratedSensorSource(1));
    jointA = new SuTalonFx(new WPI_TalonFX(Constants.Motor.ARM_JOINT_INDEX), "Joint Motor A",
        jointMotorConfig, jointSensorConfiguration);
    jointB = new SuTalonFx(new WPI_TalonFX(Constants.Motor.ARM_JOINT_FOLLOWER_INDEX),
        "Joint Motor B", jointMotorConfig, jointSensorConfiguration);
    jointB.follow(jointA);

    MotorConfiguration cascadeMotorConfig = new MotorConfiguration();
    cascadeMotorConfig.setPidProfile(Constants.Arm.CASCADE_PID_PROFILE);
    cascadeMotorConfig.setCurrentLimit(Constants.Arm.ARM_CASCADE_CURRENT_LIMIT);
    SensorConfiguration cascadeSensorConfiguration =
        new SensorConfiguration(new SensorConfiguration.IntegratedSensorSource(1));

    cascade =
        new SuSparkMax(new CANSparkMax(Constants.Motor.ARM_CASCADE_INDEX, MotorType.kBrushless),
            "Cascade Motor", cascadeMotorConfig, cascadeSensorConfiguration);

    retractCascade = false;

    stop();
    goalAngle = new TrapezoidProfile.State(jointA.outputPosition(), 0);
    currentAngle = new TrapezoidProfile.State(jointA.outputPosition(), jointA.outputVelocity());

    goalArmLength = new TrapezoidProfile.State(cascade.outputPosition(), 0);
    currentArmLength = new TrapezoidProfile.State(cascade.outputPosition(), cascade.outputVelocity());

    logger.info("Arm Initialized.");
  }

  /*
   * Reference plane for 2d coordinate has origin at joint with plane parallel to side view x and y units are feet
   */
  public void setPosition(ArmSubsystem.ArmHeight height) {
    setPosition(height.getCoordinates());
  }

  /*
   * Reference plane for 2d coordinate has origin at joint with plane parallel to side view x and y units are feet
   */
  public void setPosition(Pair<Double, Double> pair) {
    intendedPosition = pair;
    Rotation2d position = new Rotation2d(pair.getFirst(), pair.getSecond());
    goalAngle = new TrapezoidProfile.State(position.getDegrees(), 0);
    goalArmLength = new TrapezoidProfile.State(cascadeFeetToDegrees(Math.hypot(pair.getFirst(), pair.getSecond())), 0);
  }

  public void stop() {
    setPosition(getManipulatorPosition());
  }

  // TODO: Fix this, callers should not be expected to convert from polar coordinates.
  public Pair<Double, Double> getManipulatorPosition() {
    double r = 0.0444754 / 2;
    double theta = Math.toRadians(jointA.outputPosition());
    Pair<Double, Double> position = new Pair<>(r, theta);
    return position;
  }

  private double cascadeDegreesToFeet(double feet) {
    return Constants.Arm.ARM_CASCADE_STARTING_HEIGHT + (feet / Constants.Arm.ARM_CASCADE_TICKS_PER_FEET);
  }

  private double cascadeFeetToDegrees(double feet) {
    return ((feet - Constants.Arm.ARM_CASCADE_STARTING_HEIGHT) * Constants.Arm.ARM_CASCADE_TICKS_PER_FEET);
  }

  public double calcFeedForward() {
    return 0;
  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
    // When the arm is detected to be in the forbidden zone, the variable state for pause and preventExtension typically
    // goes:
    // safeMode/preventExtension = true -> pause = false (when arm is fully retracted) -> preventExtension = false (when arm is
    // moved to right position)
    refreshArmGoal();
    
    currentMechanism.update(jointA.outputPosition(), getManipulatorPosition().getFirst());
    aLogger.recordOutput("Arm/IntendedPosition", intentMechanism.asMechanism());
    aLogger.recordOutput("Arm/CurrentPosition", currentMechanism.asMechanism());
  }

  private void refreshArmGoal() {
    if (futureArmSafetyPrediction() && !safeMode) {
      // NO CHANGE
    }
    else if (!futureArmSafetyPrediction() && !safeMode) {
      safeMode = true;
      preventExtension = true;
      retractCascade = true;
      stop();
    }
    else if (retractCascade) {
      // set the arm to 0 and angle to current
      Rotation2d position = new Rotation2d(getManipulatorPosition().getFirst(), getManipulatorPosition().getSecond());
      goalAngle = new TrapezoidProfile.State(position.getDegrees(), 0);
      goalArmLength = new TrapezoidProfile.State(0, 0);

      if (isArmRetracted()) {
        retractCascade = false;
      }
    }
    else if (preventExtension) {
      goalArmLength = new TrapezoidProfile.State(0, 0);
      
      if (between(jointA.outputPosition(), goalAngle.position + Constants.Arm.ARM_JOINT_TOLERANCE, goalAngle.position - Constants.Arm.ARM_JOINT_TOLERANCE)) {
        safeMode = false;
        preventExtension = false;
      }
    }

    currentAngle = new TrapezoidProfile.State(jointA.outputPosition(), jointA.outputVelocity());
    var profileAngle = new TrapezoidProfile(constraintsAngle, goalAngle, currentAngle);
    var neededStateAngle = profileAngle.calculate(Constants.ROBOT_PERIOD);

    currentArmLength = new TrapezoidProfile.State(cascade.outputPosition(), cascade.outputVelocity());
    var profileArmLength = new TrapezoidProfile(constraintsArmLength, goalArmLength, currentArmLength);
    var neededArmLength = profileArmLength.calculate(Constants.ROBOT_PERIOD);
    
    double feedForward = calcFeedForward();
    cascade.set(ControlMode.POSITION, neededArmLength.position, feedForward);
    jointA.set(ControlMode.POSITION, neededStateAngle.position, feedForward);
  }

  private boolean futureArmSafetyPrediction() {
    double estimatedLength = cascadeDegreesToFeet(cascade.outputPosition() + (cascade.outputVelocity() * Constants.Arm.ARM_PREDICTIVE_TIMESPAN)); // 1/4 seconds should give us enough time to respond
    double estimatedAngle = jointA.outputPosition() + jointA.outputVelocity() * Constants.Arm.ARM_PREDICTIVE_TIMESPAN % 360;
    double[] cartesian = SorMath.polarToCartesian(estimatedLength, estimatedAngle);

    return isSafeFromGroundCollision(cartesian[1]) && isAngleSafe(estimatedAngle);
  }


  private boolean isSafeFromGroundCollision(double length) {
    return length > Constants.Arm.ARM_HEIGHT_FROM_BASE;
  }

  private boolean isAngleSafe(double estimatedAngle) {
    return !(between(estimatedAngle,
    Math.toRadians(Constants.Arm.ARM_MIN_ANGLE_COLLISION_A), Math.toRadians(Constants.Arm.ARM_MAX_ANGLE_COLLISION_A))
    || between(estimatedAngle, Math.toRadians(Constants.Arm.ARM_MIN_ANGLE_COLLISION_B),
    Math.toRadians(Constants.Arm.ARM_MAX_ANGLE_COLLISION_B))); 
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
    return between(
      getManipulatorPosition().getFirst(), intendedPosition.getFirst() - Constants.Arm.ARM_POSITION_TOLERANCE, intendedPosition.getFirst() + Constants.Arm.ARM_POSITION_TOLERANCE) && 
      between(getManipulatorPosition().getSecond(), intendedPosition.getSecond() - Constants.Arm.ARM_POSITION_TOLERANCE, intendedPosition.getSecond() + Constants.Arm.ARM_POSITION_TOLERANCE
    );
  }

  public enum ArmHeight {
    LOW(new Pair<Double, Double>(Constants.Arm.ARM_PRESET_LOW_X, Constants.Arm.ARM_PRESET_LOW_Y)), 
    MID_CONE(new Pair<Double, Double>(Constants.Arm.ARM_PRESET_MID_CONE_X, Constants.Arm.ARM_PRESET_MID_CONE_Y)),
    MID_CUBE(new Pair<Double, Double>(Constants.Arm.ARM_PRESET_MID_CUBE_X, Constants.Arm.ARM_PRESET_MID_CUBE_Y)),
    HIGH_CONE(new Pair<Double, Double>(Constants.Arm.ARM_PRESET_HIGH_CONE_X, Constants.Arm.ARM_PRESET_HIGH_CONE_Y)),
    HIGH_CUBE(new Pair<Double, Double>(Constants.Arm.ARM_PRESET_HIGH_CUBE_X, Constants.Arm.ARM_PRESET_HIGH_CUBE_Y)),
    TRAY(new Pair<Double, Double>(Constants.Arm.ARM_PRESET_TRAY_X, Constants.Arm.ARM_PRESET_TRAY_Y)),
    STOW(new Pair<Double, Double>(Constants.Arm.ARM_PRESET_STOW_X, Constants.Arm.ARM_PRESET_STOW_Y));

    private final Pair<Double, Double> coordinates;

    ArmHeight(Pair<Double, Double> coordinates) {
      this.coordinates = coordinates;
    }

    public Pair<Double, Double> getCoordinates() {
        return coordinates;
    }
  }
}
