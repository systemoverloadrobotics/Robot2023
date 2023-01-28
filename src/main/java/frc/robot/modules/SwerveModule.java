package frc.robot.modules;
import java.util.logging.Logger;

//Rev imports 
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

//Swerve WPIlib improts 
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//SORUtil Imports 
import frc.sorutil.SorMath;
import frc.sorutil.motor.MotorConfiguration;
import frc.sorutil.motor.SensorConfiguration;
import frc.sorutil.motor.SuSparkMax;
import frc.sorutil.motor.SensorConfiguration.ConnectedSensorType;
import frc.sorutil.motor.SuController.ControlMode;

//Robot Constants Import 
import frc.robot.Constants;

public class SwerveModule extends SubsystemBase {

	private SuSparkMax powerController;
	private SuSparkMax steeringController;
	private final String name;

	private Logger logger = Logger.getLogger(SwerveModule.class.getName());

	public SwerveModule(String name, int powerID, int steerID, double offset) {

		this.name = name;
		MotorConfiguration powerControllerConfig = new MotorConfiguration();

		powerControllerConfig.setPidProfile(Constants.Swerve.POWER_PROFILE);
		powerControllerConfig.setCurrentLimit(Constants.Swerve.SWERVE_POWER_CURRENT_LIMIT);
		powerControllerConfig.setMaxOutput(Constants.Swerve.SWERVE_POWER_MAX_OUTPUT);

		SensorConfiguration powerSensorConfig = new SensorConfiguration(
				new SensorConfiguration.IntegratedSensorSource(6.75));
		powerController = new SuSparkMax(new CANSparkMax(powerID, MotorType.kBrushless), name + " power",
				powerControllerConfig, powerSensorConfig);
		
		// Steer Controller configuration
		MotorConfiguration steerControllerConfig = new MotorConfiguration();

		steerControllerConfig.setPidProfile(Constants.Swerve.STEER_PROFILE);
		steerControllerConfig.setCurrentLimit(Constants.Swerve.SWERVE_ROTATION_CURRENT_LIMIT);
		steerControllerConfig.setMaxOutput(Constants.Swerve.SWERVE_ROTATION_MAX_OUTPUT);
		
		SensorConfiguration steerSensorConfig = new SensorConfiguration(
				new SensorConfiguration.ConnectedSensorSource(4096, 1, ConnectedSensorType.PWM_ENCODER));

		steeringController = new SuSparkMax(new CANSparkMax(steerID, MotorType.kBrushless), name + " steer",
				steerControllerConfig, steerSensorConfig);

		((CANSparkMax) steeringController.rawController())
				.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle).setZeroOffset(offset);
	}

	public String getName() {
		return this.name; 
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
