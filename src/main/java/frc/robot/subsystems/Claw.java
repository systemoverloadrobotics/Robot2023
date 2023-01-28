// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.sorutil.motor.MotorConfiguration;
import frc.sorutil.motor.SuSparkMax;
import frc.sorutil.motor.SuController.ControlMode;
import frc.robot.Constants;

public class Claw extends SubsystemBase {
  private final java.util.logging.Logger logger;
  private final Logger aLogger;

  Solenoid extendSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.Pneumatics.CLAW_SOLENOID_CHANNEL);

  private SuSparkMax rollerMotorLeft;
  private SuSparkMax rollerMotorRight;

  public Claw() {
    logger = java.util.logging.Logger.getLogger(Claw.class.getName());
    aLogger = Logger.getInstance();

    MotorConfiguration rollerControllerConfig = new MotorConfiguration();

    rollerControllerConfig.setCurrentLimit(20.0);
    rollerControllerConfig.setMaxOutput(0.8);
    
    rollerMotorLeft = new SuSparkMax(new CANSparkMax(Constants.Motor.ROLLER_LEFT, MotorType.kBrushless), "Left Roller Motor", rollerControllerConfig, 
    null);
    rollerMotorRight = new SuSparkMax(new CANSparkMax(Constants.Motor.ROLLER_RIGHT, MotorType.kBrushless), "Right Roller Motor", rollerControllerConfig, 
    null);

    logger.info("Claw Initialized.");
  }

  public void intake(double volts) {
    rollerMotorLeft.set(ControlMode.VOLTAGE, volts);
    rollerMotorRight.set(ControlMode.VOLTAGE, -volts);
  }

  public void outtake(double volts) {
    rollerMotorLeft.set(ControlMode.VOLTAGE, -volts);
    rollerMotorRight.set(ControlMode.VOLTAGE, volts);
  }

  public void stop() {
    rollerMotorLeft.stop();
    rollerMotorRight.stop();
  }

  public void extend() {
    extendSolenoid.set(true);
  }

  public void retract() {
    extendSolenoid.set(false);
  }

  public boolean isExtended() {
    return extendSolenoid.get() == true;
  }

  @Override
  public void periodic() {
    aLogger.recordOutput("Claw/Open", extendSolenoid.get());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be cawelled once per scheduler run during simulation
  }
}
