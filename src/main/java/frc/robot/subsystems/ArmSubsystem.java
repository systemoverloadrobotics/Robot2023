// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.logging.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.sorutil.motor.MotorConfiguration;
import frc.sorutil.motor.SensorConfiguration;
import frc.sorutil.motor.SuController;
import frc.sorutil.motor.SuSparkMax;

public class ArmSubsystem extends SubsystemBase {
  private final Logger logger;
  SuSparkMax joint;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    logger = Logger.getLogger(ArmSubsystem.class.getName());
    MotorConfiguration jointMotorConfig = new MotorConfiguration();
    jointMotorConfig.setCurrentLimit(Constants.Motor.ARM_JOINT_CURRENT_LIMIT);
    SensorConfiguration jointSensorConfiguration = new SensorConfiguration(new SensorConfiguration.IntegratedSensorSource(1));
    joint = new SuSparkMax(new CANSparkMax(Constants.Motor.ARM_JOINT_INDEX, MotorType.kBrushless), "Joint Motor", jointMotorConfig, jointSensorConfiguration);
  }

  public void setPosition(int position) {
    joint.set(SuController.ControlMode.POSITION, position);
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
