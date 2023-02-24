// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.logging.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.GridSelector;

public class IntelligentScoring extends SubsystemBase {
  private final Logger logger;
  private final org.littletonrobotics.junction.Logger aLogger;
  private int closestId;
  private final Vision vision;
  private final DriveTrainPoseEstimator poseEstimator;

  /** Creates a new ExampleSubsystem. */
  public IntelligentScoring() {
    vision = new Vision();
    poseEstimator = new DriveTrainPoseEstimator();
    logger = Logger.getLogger(IntelligentScoring.class.getName());
    aLogger = org.littletonrobotics.junction.Logger.getInstance();
  }

  @Override
  public void periodic() {
    closestId = GridSelector.getClosestId(vision, poseEstimator);
    aLogger.recordOutput("Scoring/ClosestId", closestId);
  }

  public int getClosestId() {
    return closestId;
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public enum ScoringLocations {
    UPPER_LEFT_CONE, UPPER_MIDDLE_CUBE, UPPER_RIGHT_CONE,
    MIDDLE_LEFT_CONE, MIDDLE_MIDDLE_CUBE, MIDDLE_RIGHT_CONE,
    HYBRID_LEFT, HYBRID_MIDDLE, HYBRID_RIGHT;
  }
}