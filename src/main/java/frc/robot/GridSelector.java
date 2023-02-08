package frc.robot;

import java.util.Arrays;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.DriveTrainPoseEstimator;
import frc.robot.subsystems.Vision;

public class GridSelector {
    private static int SELECTED_GRID_ID;
    // gives closest apriltag to the bot
    public static int getClosestId(Vision vision, DriveTrainPoseEstimator poseEstimator) {
        Pose2d currentPose = poseEstimator.getEstimatedPose();
        double closestDistance = -1;
        int closestId = 0;
        // finds the closest april tag to the bot
        for (int id : Constants.Scoring.TARGETS_PER_ALLIANCE.get(DriverStation.getAlliance())) {
            double tempIdDistance = currentPose.getTranslation().getDistance(getTagPose2d(closestId).getTranslation());
            if (tempIdDistance > closestDistance) {
                closestDistance = tempIdDistance;
                closestId = id;
            }
        }

        // checks if the closest apriltag if within the minimum distance
        Pose2d targetTagPose = getTagPose2d(closestId);
        if (currentPose.getTranslation().getDistance(targetTagPose.getTranslation()) > Constants.Scoring.MIN_AUTOMOVE_DISTANCE) {
            return -1;
        }
        SELECTED_GRID_ID = closestId;
        return closestId;
    }
    
    public static GridLocation getGridLocation(int id) {
        switch (id % 5) {
            case 1:
                return GridLocation.RIGHT;
            case 2:
                return GridLocation.MIDDLE;
            case 3:
                return GridLocation.LEFT;
            default:
                return null;
        }
    }

    public static enum GridLocation {
        RIGHT, MIDDLE, LEFT;
    }

    
    public static Pose2d getTagPose2d(int id) {
        Pose3d temPose3d = (Constants.Vision.TAG_FIELD_LAYOUT.getTagPose(id).get());
        return new Pose2d(temPose3d.getX(), temPose3d.getY(), new Rotation2d(temPose3d.getRotation().getX(), temPose3d.getRotation().getY()));
    }

    public static GridLocation getSelectedGrid() {
        return getGridLocation(SELECTED_GRID_ID);
    }
}
