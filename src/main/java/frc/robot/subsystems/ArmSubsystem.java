// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.logging.Logger;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.sorutil.SorMath;
import frc.sorutil.motor.MotorConfiguration;
import frc.sorutil.motor.SensorConfiguration;
import frc.sorutil.motor.SuController;
import frc.sorutil.motor.SuSparkMax;
import frc.sorutil.motor.SuTalonFx;
import frc.sorutil.motor.SuController.ControlMode;

public class ArmSubsystem extends SubsystemBase {
  private final Logger logger;
  private SuTalonFx jointA;
  private SuTalonFx jointB;
  private SuSparkMax cascade;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    logger = Logger.getLogger(ArmSubsystem.class.getName());
    MotorConfiguration jointMotorConfig = new MotorConfiguration();
    MotorConfiguration cascadeMotorConfig = new MotorConfiguration();
    jointMotorConfig.setCurrentLimit(Constants.Motor.ARM_JOINT_CURRENT_LIMIT);
    cascadeMotorConfig.setCurrentLimit(Constants.Motor.ARM_CASCADE_CURRENT_LIMIT);
    SensorConfiguration jointSensorConfiguration = new SensorConfiguration(new SensorConfiguration.IntegratedSensorSource(1));
    SensorConfiguration cascadeSensorConfiguration = new SensorConfiguration(new SensorConfiguration.IntegratedSensorSource(1));
    jointA = new SuTalonFx(new WPI_TalonFX(Constants.Motor.ARM_JOINT_INDEX), "Joint Motor A", jointMotorConfig, jointSensorConfiguration);
    jointB = new SuTalonFx(new WPI_TalonFX(Constants.Motor.ARM_JOINT_FOLLOWER_INDEX), "Joint Motor B", jointMotorConfig, jointSensorConfiguration);
    cascade = new SuSparkMax(new CANSparkMax(Constants.Motor.ARM_CASCADE_FOLLOWER_INDEX, MotorType.kBrushless), "Cascade Motor", jointMotorConfig, cascadeSensorConfiguration);
    jointB.follow(jointA);
  }

  /*
   * Reference plane for 2d coordinate has origin at joint with plane parallel to side view
   * x and y units are feet
   */
  public void setPosition(double x, double y) {
    double[] vec = cartesianToPolar(x, y);
    jointA.set(ControlMode.POSITION, SorMath.degreesToTicks(Math.toDegrees(vec[1]), Constants.Motor.ARM_JOINT_ENCODER_RESOLUTION));
    cascade.set(ControlMode.POSITION, lengthToTicks(vec[0]));
  }

  public double[] getElevatorPosition() {
    double r = ticksToLength(cascade.outputPosition());
    double theta = Math.toRadians(SorMath.ticksToDegrees(jointA.outputPosition(), Constants.Motor.ARM_JOINT_ENCODER_RESOLUTION));
    return new double[]{r, theta}; 
  }

  public double[] cartesianToPolar(double x, double y) {
    return new double[]{Math.sqrt(Math.pow(x, 2) + Math.pow(x, 2)), Math.atan2(y, x)};
  }

  public double[] polarToCartesian(double r, double theta) {
    return new double[]{r * Math.cos(theta), r * Math.sin(theta)};
  }

  public double lengthToTicks(double length) {
    double ticks = (length - Constants.Motor.ARM_CASCADE_STARTING_HEIGHT) * Constants.Motor.ARM_CASCADE_TICKS_PER_FEET;
    return ticks < 0 ? 0 : ticks;
  }

  public double ticksToLength(double ticks) {
    double length = (ticks / Constants.Motor.ARM_CASCADE_TICKS_PER_FEET) + Constants.Motor.ARM_CASCADE_STARTING_HEIGHT;
    return length;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
