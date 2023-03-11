// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.logging.Logger;

public class DriveTrainPoseEstimator extends SubsystemBase {
    @SuppressWarnings("unused")
    private final Logger logger;
    private Swerve swerve;
    private Vision vision;
    private SwerveDrivePoseEstimator poseEstimator;
    private final org.littletonrobotics.junction.Logger aLogger;
    private Pose2d prevPose2d;

    public DriveTrainPoseEstimator(Swerve swerve, Vision vision) {
        this.swerve = swerve;
        this.vision = vision;
        logger = Logger.getLogger(DriveTrainPoseEstimator.class.getName());
        aLogger = org.littletonrobotics.junction.Logger.getInstance();
        poseEstimator = new SwerveDrivePoseEstimator(Constants.RobotDimensions.SWERVE_DRIVE_KINEMATICS,
                swerve.getRotation2d(), swerve.getModulePositions(), new Pose2d(),
                Constants.PoseEstimation.POSE_GYRO_STD, Constants.PoseEstimation.POSE_VISION_STD);
        prevPose2d = poseEstimator.getEstimatedPosition();
    }

    public void addVisionMeasurement(Pose2d visionMeasurement, double timestampSeconds) {
        poseEstimator.addVisionMeasurement(visionMeasurement, timestampSeconds,
                Constants.PoseEstimation.POSE_VISION_STD);
    }

    public Pose2d getEstimatedPose() {
        return poseEstimator.getEstimatedPosition();
    }

    @Override
    public void periodic() {
        poseEstimator.updateWithTime(Timer.getFPGATimestamp(), swerve.getRotation2d(), swerve.getModulePositions());
        var visionPose = vision.getEstimatedGlobalPose(getEstimatedPose());
        // changes time to match time that photo is taken
        if (visionPose != null) {
            poseEstimator.addVisionMeasurement(visionPose.getFirst(), Timer.getFPGATimestamp() - visionPose.getSecond());
        }
        prevPose2d = vision.getVisionPose2d();
        aLogger.recordOutput("PoseEstimator/position", getEstimatedPose());
        if (prevPose2d != null) aLogger.recordOutput("Vision/rawPosition", prevPose2d);
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
