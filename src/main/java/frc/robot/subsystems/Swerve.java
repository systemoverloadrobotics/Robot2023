package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.modules.SwerveModule;
import com.kauailabs.navx.frc.AHRS;

public class Swerve extends SubsystemBase {
	private final SwerveModule frontLeft;
	private final SwerveModule frontRight;
	private final SwerveModule backLeft;
	private final SwerveModule backRight;

	private AHRS gyro = new AHRS(SerialPort.Port.kUSB);

	private final Timer resetTimer = new Timer();
	private boolean resetRun;

	// TODO: add the gyro for this season
	private SwerveDriveOdometry odometry = new SwerveDriveOdometry(
			Constants.RobotDimensions.SWERVE_DRIVE_KINEMATICS, new Rotation2d(0),
			new SwerveModulePosition[] {
					new SwerveModulePosition(),
					new SwerveModulePosition(),
					new SwerveModulePosition(),
					new SwerveModulePosition()
			});

	public Swerve() {
		frontLeft = new SwerveModule("Front Left", Constants.Motor.SWERVE_FRONT_LEFT_POWER,
				Constants.Motor.SWERVE_FRONT_LEFT_STEER, 2389);
		frontRight = new SwerveModule("Front Right", Constants.Motor.SWERVE_FRONT_RIGHT_POWER,
				Constants.Motor.SWERVE_FRONT_RIGHT_STEER, 805 - 2);
		backLeft = new SwerveModule("Back Left", Constants.Motor.SWERVE_BACK_LEFT_POWER,
				Constants.Motor.SWERVE_BACK_LEFT_STEER, 478 + 5);
		backRight = new SwerveModule("Back Right", Constants.Motor.SWERVE_BACK_RIGHT_POWER,
				Constants.Motor.SWERVE_BACK_RIGHT_STEER, 1421 + 30);
		gyro.reset();
		resetTimer.start();

	}

	public void stopModules() {
		frontLeft.stop();
		frontRight.stop();
		backLeft.stop();
		backRight.stop();
	}

	public SwerveModuleState[] getModuleStates() {
		return new SwerveModuleState[] {
				frontLeft.getState(),
				frontRight.getState(),
				backLeft.getState(),
				backRight.getState()
		};
	}

	public void setModuleStates(SwerveModuleState[] desiredStates) {
		frontLeft.setState(desiredStates[0]);
		frontRight.setState(desiredStates[1]);
		backLeft.setState(desiredStates[2]);
		backRight.setState(desiredStates[3]);
	}

	//gyro offset constant
	public Rotation2d getRotation2d() {
		
		return Rotation2d.fromDegrees(gyro.getYaw());
	}

	@Override
	public void periodic() {
		SmartDashboard.putData("Gyro", gyro);
		// Run reset once after a second.
		// if (!resetRun && resetTimer.get() > 1) {
		// 	gyro.reset();
		// 	resetRun = true;
		// }
		SmartDashboard.putNumber("better-gyro", gyro.getYaw()); 
		odometry.update(gyro.getRotation2d(), new SwerveModulePosition[] {
				frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(),
				backRight.getPosition()
		});

		SmartDashboard.putNumber("fl-steer", frontLeft.getState().angle.getDegrees());
	}
}
