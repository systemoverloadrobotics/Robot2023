// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.logging.Logger;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.sorutil.SorMath;
import frc.sorutil.motor.MotorConfiguration;
import frc.sorutil.motor.SensorConfiguration;
import frc.sorutil.motor.SuSparkMax;
import frc.sorutil.motor.SuTalonFx;
import frc.sorutil.motor.SuController.ControlMode;

public class ArmSubsystem extends SubsystemBase {
  private final Logger logger;

  private SuTalonFx jointA;
  private SuTalonFx jointB;
  private SuSparkMax cascade;
  private Pair<Double, Double> intendedPosition;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    logger = Logger.getLogger(ArmSubsystem.class.getName());

    MotorConfiguration jointMotorConfig = new MotorConfiguration();

    jointMotorConfig.setPidProfile(Constants.Arm.ARM_PID_PROFILE);
    jointMotorConfig.setCurrentLimit(Constants.Arm.ARM_JOINT_CURRENT_LIMIT);
    SensorConfiguration jointSensorConfiguration = new SensorConfiguration(
        new SensorConfiguration.IntegratedSensorSource(1));
    jointA = new SuTalonFx(new WPI_TalonFX(Constants.Motor.ARM_JOINT_INDEX), "Joint Motor A", jointMotorConfig,
        jointSensorConfiguration);
    jointB = new SuTalonFx(new WPI_TalonFX(Constants.Motor.ARM_JOINT_FOLLOWER_INDEX), "Joint Motor B", jointMotorConfig,
        jointSensorConfiguration);
    jointB.follow(jointA);

    MotorConfiguration cascadeMotorConfig = new MotorConfiguration();
    cascadeMotorConfig.setPidProfile(Constants.Arm.CASCADE_PID_PROFILE);
    cascadeMotorConfig.setCurrentLimit(Constants.Arm.ARM_CASCADE_CURRENT_LIMIT);
    SensorConfiguration cascadeSensorConfiguration = new SensorConfiguration(
        new SensorConfiguration.IntegratedSensorSource(1));

    cascade = new SuSparkMax(new CANSparkMax(Constants.Motor.ARM_CASCADE_INDEX, MotorType.kBrushless), "Cascade Motor",
        cascadeMotorConfig, cascadeSensorConfiguration);

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
    double[] vec = SorMath.cartesianToPolar(intendedPosition.getFirst(), intendedPosition.getSecond());
    jointA.set(ControlMode.POSITION, Math.toDegrees(vec[1]));
    cascade.set(ControlMode.POSITION, vec[0]);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}