// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.ArrayList;
import edu.wpi.first.math.Pair;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.RobotPoseEstimator;
import org.photonvision.RobotPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Vision extends SubsystemBase {
    private final java.util.logging.Logger logger;
    private final Logger aLogger;

    private final PhotonCamera camera;
    private final RobotPoseEstimator robotPoseEstimator;
    private Pose2d visonPose2d;

    private PhotonPipelineResult results;

    public Vision() { // IOException is thrown when the file for April Tag field layout isn't found
        logger = java.util.logging.Logger.getLogger(Vision.class.getName());
        aLogger = Logger.getInstance();

        Transform3d robotCam = new Transform3d(Constants.Vision.CAMERA_POSITION, Constants.Vision.CAMERA_ROTATION);
        camera = new PhotonCamera("Camera");
        results = camera.getLatestResult();
        var cameraList = new ArrayList<Pair<PhotonCamera, Transform3d>>();
        cameraList.add(new Pair<PhotonCamera, Transform3d>(camera, robotCam));
        robotPoseEstimator = new RobotPoseEstimator(Constants.Vision.TAG_FIELD_LAYOUT,
                PoseStrategy.CLOSEST_TO_REFERENCE_POSE, cameraList);

        logger.info("Vision Initialized.");
    }

    /**
     * 
     * @return A boolean that signifies whether or not the camera recognizes any apriltags
     */
    public boolean targetsExist() {
        return camera.getLatestResult().hasTargets();
    }

    public PhotonTrackedTarget getBestTarget() {
        return camera.isConnected() ? results.getBestTarget() : null;
    }

    public List<PhotonTrackedTarget> getTargets() {
        return camera.isConnected() ? results.getTargets() : null;
    }


    /**
     * Returns the estimated pose from the camera based on at least one AprilTag being visible. In the case where no
     * target is visible, this returns null.
     *
     * @return A pair of Pose2D (representing the position on the field) and a double representing the delay in
     *         milliseconds since the image was acquired.
     */
    public Pair<Pose2d, Double> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        robotPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        double currentTime = Timer.getFPGATimestamp();
        var result = robotPoseEstimator.update();
        if (result.isPresent() && result.get().getFirst() != null) {
            Pose2d pose = result.get().getFirst().toPose2d();
            visonPose2d = pose;
            var deltaTms = currentTime - result.get().getSecond();
            return new Pair<Pose2d, Double>(pose, deltaTms);
        }
        return null;
    }

    public Pose2d getVisionPose2d() {
        return visonPose2d;
    }


    @Override
    public void periodic() {
        if (camera.isConnected()) {
            results = camera.getLatestResult();
        }

        aLogger.recordOutput("Vision/HasTarget", targetsExist());
        aLogger.recordOutput("Vision/CameraConnected", camera.isConnected());

        try {
            aLogger.recordOutput("Vision/VisionPose", robotPoseEstimator.getReferencePose());
        } catch (Exception e) {
            
        }
    }
}
