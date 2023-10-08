package frc.robot.subsystems;

// Rev imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder;
// Swerve WPIlib improts
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// Robot Constants Import
import frc.robot.Constants;
// SORUtil Imports
import frc.sorutil.SorMath;
import frc.sorutil.motor.MotorConfiguration;
import frc.sorutil.motor.SensorConfiguration;
import frc.sorutil.motor.SensorConfiguration.ConnectedSensorType;
import frc.sorutil.motor.SuController.ControlMode;
import frc.sorutil.motor.SuSparkMax;
import org.littletonrobotics.junction.Logger;

public class SwerveModule extends SubsystemBase {

    private java.util.logging.Logger logger = java.util.logging.Logger.getLogger(SwerveModule.class.getName());
    private final Logger aLogger;

    private SuSparkMax powerController;
    private SuSparkMax steeringController;
    private final String name;

    public SwerveModule(String name, int powerID, int steerID, double offset) {
        this.name = name;
        aLogger = Logger.getInstance();

        MotorConfiguration powerControllerConfig = new MotorConfiguration();

        powerControllerConfig.setPidProfile(Constants.Swerve.POWER_PROFILE);
        powerControllerConfig.setCurrentLimit(Constants.Swerve.SWERVE_POWER_CURRENT_LIMIT);
        powerControllerConfig.setMaxOutput(Constants.Swerve.SWERVE_POWER_MAX_OUTPUT);

        SensorConfiguration powerSensorConfig =
                new SensorConfiguration(new SensorConfiguration.IntegratedSensorSource(6.75));
        powerController = new SuSparkMax(new CANSparkMax(powerID, MotorType.kBrushless), name + " Power",
                powerControllerConfig, powerSensorConfig);

        // Steer Controller configuration
        MotorConfiguration steerControllerConfig = new MotorConfiguration();

        steerControllerConfig.setPidProfile(Constants.Swerve.STEER_PROFILE);
        steerControllerConfig.setCurrentLimit(Constants.Swerve.SWERVE_ROTATION_CURRENT_LIMIT);
        steerControllerConfig.setMaxOutput(Constants.Swerve.SWERVE_ROTATION_MAX_OUTPUT);

        SensorConfiguration steerSensorConfig = new SensorConfiguration(
                new SensorConfiguration.ConnectedSensorSource(4096, 1, ConnectedSensorType.PWM_ENCODER));

        steeringController = new SuSparkMax(new CANSparkMax(steerID, MotorType.kBrushless), name + " Steer",
                steerControllerConfig, steerSensorConfig);

        ((CANSparkMax) steeringController.rawController()).getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle)
                .setZeroOffset(offset);

        stateName = "Swerve/" + name.replace(" ", "") + "/ModuleState";

        logger.info("\tModule " + name + " Initialized.");
    }

    public String getName() {
        return this.name;
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(this.powerController.outputVelocity(),
                Rotation2d.fromDegrees(this.steeringController.outputPosition()));
    }

    public void setState(SwerveModuleState state) {
        //state = SwerveModuleState.optimize(state, getState().angle);
        powerController.set(ControlMode.VELOCITY,
                SorMath.speedMetersPerSecondToRevsPerMinute(4, state.speedMetersPerSecond));
        
        steeringController.set(ControlMode.POSITION, state.angle.getDegrees());
        aLogger.recordOutput(stateName + "SteerPos", steeringController.outputPosition());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(SorMath.degreesToMeters(4, powerController.outputPosition()),
                new Rotation2d(steeringController.outputPosition()));
    }

    public void stop() {
        powerController.stop();
        steeringController.stop();
    }

    private final String stateName;

    @Override
    public void periodic() {
        aLogger.recordOutput(stateName, getState());
    }
}
