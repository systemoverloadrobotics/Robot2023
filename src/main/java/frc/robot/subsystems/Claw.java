// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.sorutil.motor.MotorConfiguration;
import frc.sorutil.motor.PidProfile;
import frc.sorutil.motor.SensorConfiguration;
import frc.sorutil.motor.SuSparkMax;
import frc.sorutil.motor.SuController.ControlMode;
import frc.robot.Constants;

public class Claw extends SubsystemBase {
    private final java.util.logging.Logger logger;
    private final Logger aLogger;

    private SuSparkMax rollerMotorLeft;
    private SuSparkMax rollerMotorRight;

    private String lastState = "stop";

    public Claw() {
        logger = java.util.logging.Logger.getLogger(Claw.class.getName());
        aLogger = Logger.getInstance();
        MotorConfiguration rollerControllerConfig = new MotorConfiguration();
        rollerControllerConfig.setPidProfile(new PidProfile(0.0001, 0, 0));

        rollerControllerConfig.setCurrentLimit(Constants.Claw.CLAW_CURRENT_LIMIT);
        rollerControllerConfig.setMaxOutput(0.8);
        System.out.print("hi");
        SensorConfiguration sensorConfiguration =
                new SensorConfiguration(new SensorConfiguration.IntegratedSensorSource(10));

        rollerMotorLeft = new SuSparkMax(new CANSparkMax(Constants.Motor.ROLLER_LEFT, MotorType.kBrushless),
                "Left Roller Motor", rollerControllerConfig, sensorConfiguration);
        rollerMotorRight = new SuSparkMax(new CANSparkMax(Constants.Motor.ROLLER_RIGHT, MotorType.kBrushless),
                "Right Roller Motor", rollerControllerConfig, sensorConfiguration);
        logger.info("Claw Initialized.");
    }

    public void intake() {
        rollerMotorLeft.set(ControlMode.VELOCITY, -Constants.Claw.CLAW_VELOCITY);
        rollerMotorRight.set(ControlMode.VELOCITY, Constants.Claw.CLAW_VELOCITY);

        lastState = "intake";
    }

    public void outtake() {
        rollerMotorLeft.set(ControlMode.VELOCITY, Constants.Claw.CLAW_VELOCITY_OUT);
        rollerMotorRight.set(ControlMode.VELOCITY, -Constants.Claw.CLAW_VELOCITY_OUT);

        lastState = "outtake";
    }

    public void stop() {
        rollerMotorLeft.set(ControlMode.VELOCITY, 0);
        rollerMotorRight.set(ControlMode.VELOCITY, 0);

        lastState = "stop";
    }

    @Override
    public void periodic() {
        aLogger.recordOutput("Claw/LastState", lastState);
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
