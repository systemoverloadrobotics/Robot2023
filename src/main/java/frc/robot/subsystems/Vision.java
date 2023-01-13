// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import java.util.List;
import java.util.ArrayList; 
import edu.wpi.first.math.Pair;
import java.util.logging.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.RobotPoseEstimator;
import org.photonvision.RobotPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import lombok.SneakyThrows;


public class Vision extends SubsystemBase {
  private final Logger logger;
  private PhotonCamera camera;
  private PhotonPipelineResult results;
  AprilTagFieldLayout aprilTagFieldLayout; 
  RobotPoseEstimator robotPoseEstimator;

  @SneakyThrows
  public Vision() {
    Transform3d robotCam = new Transform3d(new Translation3d(Constants.Vision.CAMERA_POSITION_X,Constants.Vision.CAMERA_POSITION_Y,Constants.Vision.CAMERA_POSITION_Z), new Rotation3d(Constants.Vision.CAMERA_ROTATION_ROLL,Constants.Vision.CAMERA_ROTATION_PITCH,Constants.Vision.CAMERA_ROTATION_YAW));
    logger = Logger.getLogger(ExampleSubsystem.class.getName());
    camera = new PhotonCamera("Camera");
    results = camera.getLatestResult();
    aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource("/edu/wpi/first/apriltag/2023-chargedup.json");
    var cameraList = new ArrayList<Pair<PhotonCamera,  Transform3d>>();
    cameraList.add(new Pair<PhotonCamera, Transform3d>(camera, robotCam)); 
    robotPoseEstimator = new RobotPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, cameraList);
  } 
  
  /**
   * Returns whether the pipeline has targets.
   * Returns whether the camera is sensing any apriltags
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
   * Returns the estimated pose from the camera based on at least one AprilTag
   * being visible. In the case where no target is visible, this returns null.
   *
   * @return A pair of Pose2D (representing the position on the field) and a
   * double representing the delay in milliseconds since the image was acquired.
   */
  public Pair<Pose2d, Double> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    robotPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    double currentTime = Timer.getFPGATimestamp(); 
    
    var result = robotPoseEstimator.update();


    if (result.isPresent()) {
        var pose = result.get().getFirst().toPose2d();
        var deltaTms = currentTime - result.get().getSecond();
        return new Pair<Pose2d, Double>(pose, deltaTms);
    } 
    return null; 
  }

  
  @Override
  public void periodic() {
      if(camera.isConnected()) results = camera.getLatestResult(); 
  }
}