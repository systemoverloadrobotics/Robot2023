// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.logging.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.sorutil.motor.MotorConfiguration;
import frc.sorutil.motor.PidProfile;
import frc.sorutil.motor.SensorConfiguration;
import frc.sorutil.motor.SuSparkMax;
import frc.sorutil.motor.SuController.ControlMode;

public class Claw extends SubsystemBase {

  Solenoid extendSolenoidLeft = new Solenoid(PneumaticsModuleType.CTREPCM, 1); // TODO: change solenoid channels
  Solenoid extendSolenoidRight = new Solenoid(PneumaticsModuleType.CTREPCM, 2); 

  private SuSparkMax rollerMotorLeft;
  private SuSparkMax rollerMotorRight;

  private final Logger logger;

  public Claw() {
    logger = Logger.getLogger(Claw.class.getName());

    MotorConfiguration rollerControllerConfig = new MotorConfiguration();
    
    // TODO: adjust the PID values for roller controller, along with ampage/voltage
		rollerControllerConfig.setPidProfile(new PidProfile(0.01, 0.0, 0.001));
		rollerControllerConfig.setCurrentLimit(20.0);
		rollerControllerConfig.setMaxOutput(0.8);


    // TODO: implement actual gear ratio
    SensorConfiguration rollerSensorConfig = new SensorConfiguration(
      new SensorConfiguration.IntegratedSensorSource(1)
    );

    // TODO: Update Device IDs
    rollerMotorLeft = new SuSparkMax(new CANSparkMax(0, MotorType.kBrushless), "Left Roller Motor", rollerControllerConfig, 
    rollerSensorConfig);
    rollerMotorRight = new SuSparkMax(new CANSparkMax(1, MotorType.kBrushless), "Right Roller Motor", rollerControllerConfig, 
    rollerSensorConfig);
  }

  public void intake(double volts) {
    rollerMotorLeft.set(ControlMode.VOLTAGE, volts);
    rollerMotorRight.set(ControlMode.VOLTAGE, -volts);
  }

  public void outtake(double volts) {
    rollerMotorLeft.set(ControlMode.VOLTAGE, -volts);
    rollerMotorRight.set(ControlMode.VOLTAGE, volts);
  }

  // TODO: There's probably an in-built "motor-stop" function that looks better
  public void stop() {
    rollerMotorLeft.set(ControlMode.VOLTAGE, 0);
    rollerMotorRight.set(ControlMode.VOLTAGE, 0);
  }

  @Override
  public void periodic() {
    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
