// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.Pair;
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

    logger.info("Arm Initialized.");
  }

  /*
   * Reference plane for 2d coordinate has origin at joint with plane parallel to side view x and y units are feet
   */
  public void setPosition(Pair<Double, Double> pair) {
    intendedPosition = pair;
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

  public double cascadeTicksToFeet(double ticks) {
    return Constants.Arm.ARM_CASCADE_STARTING_HEIGHT
        + (ticks / Constants.Arm.ARM_CASCADE_TICKS_PER_FEET);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // TODO: This should be changed to run a constant program that seeks the correct
    // position while keeping the arm safe.

    // When the arm is detected to be in the forbidden zone, the variable state for pause and preventExtension typically
    // goes:
    // pause/preventExtension = true -> pause = false (when arm is fully retracted) -> preventExtension = false (when arm is
    // moved to right position)
    double[] vec =
        SorMath.cartesianToPolar(intendedPosition.getFirst(), intendedPosition.getSecond());
    // if (retractCascade) {
    //   jointA.set(ControlMode.VELOCITY, 0);
    //   cascade.set(ControlMode.POSITION, 0);
    //   if (cascade.outputPosition() < Constants.Arm.ARM_CASCADE_TOLERANCE) { // code to check if the arm is fully retracted
    //     retractCascade = false;
    //   }
    //   intentMechanism.update(0, 0);
    // } else if (preventExtension) {
    //   jointA.set(ControlMode.POSITION, Math.toDegrees(vec[1]));
    //   cascade.set(ControlMode.POSITION, 0);
    //   if (Math.abs(jointA.outputPosition() - SorMath.degreesToTicks(Math.toDegrees(vec[1]), 4096))
    //       % 4096 < Constants.Arm.ARM_JOINT_TOLERANCE) { // checks if joint is in the right position
    //     preventExtension = false;
    //   }
    //   intentMechanism.update(Math.toDegrees(vec[1]), 0);
    // } else if (intendedPosition != null) {
    //   // Running a predictive program to check whether collision is imminent in the near future
    //   double estimatedLength = cascadeTicksToFeet(cascade.outputPosition()
    //       + (cascade.outputVelocity() * Constants.Arm.ARM_PREDICTIVE_TIMESPAN)); // 1/4 seconds should give us enough time to respond
    //   double estimatedAngle =
    //       SorMath
    //           .degreesToTicks(
    //               (jointA.outputPosition()
    //                   + jointA.outputVelocity() * Constants.Arm.ARM_PREDICTIVE_TIMESPAN) % 360,
    //               4096);
    //   double[] cartesian = SorMath.polarToCartesian(estimatedLength, estimatedAngle);

    //   if (cartesian[1] <= Constants.Arm.ARM_HEIGHT_FROM_BASE) { // inside box bound
    //     if (between(estimatedAngle, Constants.Arm.ARM_MIN_ANGLE_COLLISION_A,
    //         Constants.Arm.ARM_MAX_ANGLE_COLLISION_A)
    //         || between(estimatedAngle, Constants.Arm.ARM_MIN_ANGLE_COLLISION_B,
    //             Constants.Arm.ARM_MAX_ANGLE_COLLISION_B))
    //       retractCascade = true;
    //     preventExtension = true;
    //   } else if (cartesian[1] <= Constants.Arm.ARM_HEIGHT_FROM_GROUND) { // lower than ground
    //     retractCascade = true;
    //     preventExtension = true;
    //   } else { // All good to go!
    //     jointA.set(ControlMode.POSITION, Math.toDegrees(vec[1]));
    //     cascade.set(ControlMode.POSITION, vec[0]);
    //     intentMechanism.update(Math.toDegrees(vec[1]), vec[0]);
    //   }
    // }

    currentMechanism.update(jointA.outputPosition(), getManipulatorPosition().getFirst());

    aLogger.recordOutput("Arm/IntendedPosition", intentMechanism.asMechanism());
    aLogger.recordOutput("Arm/CurrentPosition", currentMechanism.asMechanism());
  }

  private void arm() {
    double[] desiredPosition =
        SorMath.cartesianToPolar(intendedPosition.getFirst(), intendedPosition.getSecond());
    if (!isArmReady()) {
      cascade.set(ControlMode.POSITION, desiredPosition[0]);
      jointA.set(ControlMode.POSITION, desiredPosition[1]);
    }

  }

  private boolean isArmReady() {
    double estimatedLength = cascadeTicksToFeet(cascade.outputPosition()
        + (cascade.outputVelocity() * Constants.Arm.ARM_PREDICTIVE_TIMESPAN)); // 1/4 seconds should give us enough time to respond
    double estimatedAngle = SorMath.degreesToTicks(
        (jointA.outputPosition() + jointA.outputVelocity() * Constants.Arm.ARM_PREDICTIVE_TIMESPAN)
            % 360,
        4096);
    double[] cartesian = SorMath.polarToCartesian(estimatedLength, estimatedAngle);

    return isCascadeSafe(cartesian) && isArmPositionSafe(estimatedAngle);
  }


  private boolean isCascadeSafe(double[] cartesian) {
    return cartesian[1] >= Constants.Arm.ARM_HEIGHT_FROM_BASE;
  }

  private boolean isArmPositionSafe(double estimatedAngle) {
    return between(estimatedAngle,
    Constants.Arm.ARM_MIN_ANGLE_COLLISION_A, Constants.Arm.ARM_MAX_ANGLE_COLLISION_A)
    || between(estimatedAngle, Constants.Arm.ARM_MIN_ANGLE_COLLISION_B,
        Constants.Arm.ARM_MAX_ANGLE_COLLISION_B); 
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public static boolean between(double in, double low, double high) {
    return in > low && in < high;
  }

  public static boolean withinRange(Pair<Double, Double> in, Pair<Double, Double> origin, double xdiff, double ydiff) {
    return between(in.getFirst(), origin.getFirst() - xdiff, origin.getFirst() + xdiff) && between(in.getSecond(), origin.getSecond() - ydiff, origin.getSecond() + ydiff);
  }

  public enum ArmHeight {
    LOW(new Pair<Double, Double>(Constants.Arm.ARM_PRESET_LOW_X, Constants.Arm.ARM_PRESET_LOW_Y)), 
    MID_CONE(new Pair<Double, Double>(Constants.Arm.ARM_PRESET_MID_CONE_X, Constants.Arm.ARM_PRESET_MID_CONE_Y)),
    MID_CUBE(new Pair<Double, Double>(Constants.Arm.ARM_PRESET_MID_CUBE_X, Constants.Arm.ARM_PRESET_MID_CUBE_Y)),
    HIGH_CONE(new Pair<Double, Double>(Constants.Arm.ARM_PRESET_HIGH_CONE_X, Constants.Arm.ARM_PRESET_HIGH_CONE_Y)),
    HIGH_CUBE(new Pair<Double, Double>(Constants.Arm.ARM_PRESET_HIGH_CUBE_X, Constants.Arm.ARM_PRESET_HIGH_CUBE_Y)),
    TRAY(new Pair<Double, Double>(Constants.Arm.ARM_PRESET_TRAY_X, Constants.Arm.ARM_PRESET_TRAY_Y)),
    STOW(new Pair<Double, Double>(Constants.Arm.ARM_PRESET_STOW_X, Constants.Arm.ARM_PRESET_STOW_Y));

    private Pair<Double, Double> coordinates;

    ArmHeight(Pair<Double, Double> coordinates) {
      this.coordinates = coordinates;
    }

    public Pair<Double, Double> getCoordinates() {
        return coordinates;
    }
  }
}
