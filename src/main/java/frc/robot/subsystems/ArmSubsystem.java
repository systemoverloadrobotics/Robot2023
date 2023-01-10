// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.logging.Logger;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.sorutil.SorMath;
import frc.sorutil.motor.MotorConfiguration;
import frc.sorutil.motor.SensorConfiguration;
import frc.sorutil.motor.SuController;
import frc.sorutil.motor.SuTalonFx;

public class ArmSubsystem extends SubsystemBase {
  private final Logger logger;
  SuTalonFx jointA;
  SuTalonFx jointB;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    logger = Logger.getLogger(ArmSubsystem.class.getName());
    MotorConfiguration jointMotorConfig = new MotorConfiguration();
    jointMotorConfig.setCurrentLimit(Constants.Motor.ARM_JOINT_CURRENT_LIMIT);
    SensorConfiguration jointSensorConfiguration = new SensorConfiguration(new SensorConfiguration.IntegratedSensorSource(1));
    jointA = new SuTalonFx(new WPI_TalonFX(Constants.Motor.ARM_JOINT_INDEX), "Joint Motor A", jointMotorConfig, jointSensorConfiguration);
    jointB = new SuTalonFx(new WPI_TalonFX(Constants.Motor.ARM_JOINT_FOLLOWER_INDEX), "Joint Motor B", jointMotorConfig, jointSensorConfiguration);
    jointB.follow(jointA);
  }

  public void setPosition(int position) {
    jointA.set(SuController.ControlMode.POSITION, position);
  }

  /** 
   * Sets the ArmSubsystem's angle
   * The zero is the angle where the arm is at its highest, with clockwise movement increasing the angle
   * @param angle angle that the arm needs to be set to in degrees
   */
  public void setAngle(int angle) {
    setPosition((int) Math.round(SorMath.degreesToTicks(angle, Constants.Motor.ARM_JOINT_ENCODER_RESOLUTION)));
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
