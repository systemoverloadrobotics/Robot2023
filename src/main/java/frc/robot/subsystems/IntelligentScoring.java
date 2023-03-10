// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.logging.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.GridSelector;
import frc.robot.GridSelector.GridLocation;


public class IntelligentScoring extends SubsystemBase {
  private final Logger logger;
  private final org.littletonrobotics.junction.Logger aLogger;
  private int closestId;
  private final Vision vision;
  private final DriveTrainPoseEstimator poseEstimator;

  /** Creates a new ExampleSubsystem. */
  public IntelligentScoring(Vision vision, DriveTrainPoseEstimator poseEstimator) {
    this.vision = vision;
    this.poseEstimator = poseEstimator;
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

  public GridLocation getGridLocation() {
    return GridSelector.getGridLocation(GridSelector.getClosestId(vision, poseEstimator));
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public enum ScoringLocations {
    //@formatter:off
    UPPER_LEFT_CONE,
    UPPER_MIDDLE_CUBE,
    UPPER_RIGHT_CONE,
    MIDDLE_LEFT_CONE,
    MIDDLE_MIDDLE_CUBE,
    MIDDLE_RIGHT_CONE,
    HYBRID_LEFT,
    HYBRID_MIDDLE,
    HYBRID_RIGHT;
    //@formatter:on
  }

  public static enum GridOffset {
    LEFT, CENTER, RIGHT;
  }

  /**
   * 
   * @param scoringLocation
   * @return offset to move either right to left based on selected scoring location
   */
  public GridOffset getGridOffset(ScoringLocations scoringLocation) {
    int location = scoringLocation.ordinal() + 1;
    if(location % 3 == 0) {
      return GridOffset.LEFT;
    }
    if(location % 3 == 2) {
      return GridOffset.RIGHT;
    }
    return GridOffset.CENTER;
  }
}