package frc.robot.modules;

import java.util.logging.Logger;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.sorutil.SorMath;
import frc.sorutil.motor.MotorConfiguration;
import frc.sorutil.motor.PidProfile;
import frc.sorutil.motor.SensorConfiguration;
import frc.sorutil.motor.SuController;
import frc.sorutil.motor.SuSparkMax;
import frc.sorutil.motor.SensorConfiguration.CanCoder;
import frc.sorutil.motor.SensorConfiguration.ConnectedSensorType;
import frc.sorutil.motor.SuController.ControlMode;

public class SwerveModule extends SubsystemBase {

	private SuSparkMax powerController;
	private SuSparkMax steeringController;

	private Logger logger = Logger.getLogger(SwerveModule.class.getName());

	public SwerveModule(String name, int powerID, int steerID, double offset) {

		// Power controller configuration
		MotorConfiguration powerControllerConfig = new MotorConfiguration();

		// TODO: adjust the PID values for power controller
		powerControllerConfig.setPidProfile(new PidProfile(0.01, 0.0, 0.001));
		powerControllerConfig.setCurrentLimit(20.0);
		powerControllerConfig.setMaxOutput(0.6);

		// TODO: find gear ratio for outputOffset
		SensorConfiguration powerSensorConfig = new SensorConfiguration(
				new SensorConfiguration.IntegratedSensorSource(1));
		powerController = new SuSparkMax(new CANSparkMax(powerID, MotorType.kBrushless), name + " power",
				powerControllerConfig, powerSensorConfig);

		// Steer Controller configuration
		MotorConfiguration steerControllerConfig = new MotorConfiguration();

		// TODO: adjust the PID values for power controller
		steerControllerConfig.setPidProfile(new PidProfile(0.01, 0.0, 0.001));
		steerControllerConfig.setCurrentLimit(20.0);
		steerControllerConfig.setMaxOutput(0.8);
		// SensorConfiguration steerSensorConfig = new SensorConfiguration(
		// 		new SensorConfiguration.IntegratedSensorSource(1));
		SensorConfiguration steerSensorConfig = new SensorConfiguration(
				new SensorConfiguration.ConnectedSensorSource(0, 1, ConnectedSensorType.PWM_ENCODER));

		steeringController = new SuSparkMax(new CANSparkMax(steerID, MotorType.kBrushless), name + " steer",
				steerControllerConfig, steerSensorConfig);

		((CANSparkMax) steeringController.rawController())
				.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle).setZeroOffset(offset);
	}

	public SwerveModuleState getState() {
		return new SwerveModuleState(this.powerController.outputVelocity(),
				Rotation2d.fromDegrees(this.steeringController.outputPosition()));
	}

	public void setState(SwerveModuleState state) {
		state = SwerveModuleState.optimize(state, getState().angle);
		powerController.set(ControlMode.VELOCITY,
				SorMath.speedMetersPerSecondToRevsPerMinute(4, state.speedMetersPerSecond));
		steeringController.set(ControlMode.POSITION, state.angle.getDegrees());
	}

	public SwerveModulePosition getPosition() {
		return new SwerveModulePosition(
			SorMath.degreesToMeters(2, powerController.outputPosition()),
			new Rotation2d(steeringController.outputPosition()));
	}

	public void stop() {
		powerController.set(ControlMode.PERCENT_OUTPUT, 0);
		this.steeringController.set(ControlMode.PERCENT_OUTPUT, 0);
	}
}
