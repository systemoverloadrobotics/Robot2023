// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.Pair;
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
import frc.sorutil.motor.SuController.ControlMode;
import frc.sorutil.motor.SuSparkMax;
import frc.sorutil.motor.SuTalonFx;
import org.littletonrobotics.junction.Logger;

public class ArmSubsystem extends SubsystemBase {

  private final java.util.logging.Logger logger;
  private final Logger aLogger;

  private SuTalonFx jointA;
  private SuTalonFx jointB;
  private SuSparkMax cascade;
  private Pair<Double, Double> intendedPosition;

  private static class ArmModel {
    private final Mechanism2d mech = new Mechanism2d(2, 2);
    private final MechanismLigament2d elevatorExtension;
    private final MechanismLigament2d armBack;

    public ArmModel(String name) {
      var root = mech.getRoot(name, 1, 0);
      var upright = root
          .append(
              new MechanismLigament2d("upright", Constants.Arm.ARM_PIVOT_Y, 90, 4, new Color8Bit(Color.kDarkKhaki)));
      armBack = upright.append(
          new MechanismLigament2d("armBack", Constants.Arm.MIN_ARM_LENGTH, 90, 8, new Color8Bit(Color.kAquamarine)));
      armBack.append(
          new MechanismLigament2d("arm", Constants.Arm.MIN_ARM_LENGTH * 2, 180, 8, new Color8Bit(Color.kAquamarine)));
      elevatorExtension = armBack
          .append(new MechanismLigament2d("elevatorExtension", 0, 180, 4, new Color8Bit(Color.kAquamarine)));
      elevatorExtension.append(
          new MechanismLigament2d("elevator", Constants.Arm.MIN_ARM_LENGTH * 2, 0, 4, new Color8Bit(Color.kGreen)));
    }

    /**
     * update changes the position of the arm model to match the arguments passed.
     *
     * The first argument should be angle of the arm in degrees measured from
     * straight out as 0, and the second argument should be the length of the arm
     * from the pivot, accounting for the minimum length of the arm.
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
    SensorConfiguration jointSensorConfiguration = new SensorConfiguration(
      new SensorConfiguration.IntegratedSensorSource(1)
    );
    jointA =
      new SuTalonFx(
        new WPI_TalonFX(Constants.Motor.ARM_JOINT_INDEX),
        "Joint Motor A",
        jointMotorConfig,
        jointSensorConfiguration
      );
    jointB =
      new SuTalonFx(
        new WPI_TalonFX(Constants.Motor.ARM_JOINT_FOLLOWER_INDEX),
        "Joint Motor B",
        jointMotorConfig,
        jointSensorConfiguration
      );
    jointB.follow(jointA);

    MotorConfiguration cascadeMotorConfig = new MotorConfiguration();
    cascadeMotorConfig.setPidProfile(Constants.Arm.CASCADE_PID_PROFILE);
    cascadeMotorConfig.setCurrentLimit(Constants.Arm.ARM_CASCADE_CURRENT_LIMIT);
    SensorConfiguration cascadeSensorConfiguration = new SensorConfiguration(
      new SensorConfiguration.IntegratedSensorSource(1)
    );

    cascade =
      new SuSparkMax(
        new CANSparkMax(
          Constants.Motor.ARM_CASCADE_INDEX,
          MotorType.kBrushless
        ),
        "Cascade Motor",
        cascadeMotorConfig,
        cascadeSensorConfiguration
      );

    logger.info("Arm Initialized.");
  }

  /*
   * Reference plane for 2d coordinate has origin at joint with plane parallel to
   * side view
   * x and y units are feet
   */
  public void setPosition(Pair<Double, Double> pair) {
    intendedPosition = pair;
  }

  // TODO: Fix this, callers should not be expected to convert from polar coordinates.
  public Pair<Double, Double> getManipulatorPosition() {
    double r = 0.0444754 / 2;
    double theta = Math.toRadians(jointA.outputPosition());
    Pair<Double, Double> position = new Pair<>(r, theta);
    return position;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // TODO: This should be changed to run a constant program that seeks the correct
    // position while keeping the arm safe.
    if (intendedPosition != null) {
      double[] vec = SorMath.cartesianToPolar(intendedPosition.getFirst(), intendedPosition.getSecond());
      jointA.set(ControlMode.POSITION, Math.toDegrees(vec[1]));
      cascade.set(ControlMode.POSITION, vec[0]);
      intentMechanism.update(Math.toDegrees(vec[1]), vec[0]);
    }

    currentMechanism.update(jointA.outputPosition(), getManipulatorPosition().getFirst());

    aLogger.recordOutput("Arm/IntendedPosition", intentMechanism.asMechanism());
    aLogger.recordOutput("Arm/CurrentPosition", currentMechanism.asMechanism());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
