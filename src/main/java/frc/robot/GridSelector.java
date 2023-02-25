package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.DriveTrainPoseEstimator;
import frc.robot.subsystems.Vision;

public class GridSelector {
    private int SELECTED_GRID_ID;
    
    /**
     * @return closest april tag with respect to alliance
     **/
    public int getClosestId(Vision vision, DriveTrainPoseEstimator poseEstimator) {
        Pose2d currentPose = poseEstimator.getEstimatedPose();
        double closestDistance = -1;
        int closestId = 0;
        // finds the closest Grid on your alliance
        for (int id : Constants.Scoring.TARGETS_PER_ALLIANCE.get(DriverStation.getAlliance())) {
            double tempIdDistance = currentPose.getTranslation().getDistance(getTagPose2d(closestId).getTranslation());
            if (tempIdDistance > closestDistance) {
                closestDistance = tempIdDistance;
                closestId = id;
            }
        }

        /* 
         * checks if the closest apriltag if within the minimum distance  
         */
        Pose2d targetTagPose = getTagPose2d(closestId);
        if (currentPose.getTranslation().getDistance(targetTagPose.getTranslation()) > Constants.Scoring.MAX_AUTOMOVE_DISTANCE) {
            return -1;
        }
        SELECTED_GRID_ID = closestId;
        return closestId;
    }
    
    /**
     * Orientation from the center of the field facing grids
     **/
    public GridLocation getGridLocation(int id) {
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

    public enum GridLocation {
        RIGHT, MIDDLE, LEFT;
    }

    
    public Pose2d getTagPose2d(int id) {
        Pose3d temPose3d = (Constants.Vision.TAG_FIELD_LAYOUT.getTagPose(id).get());
        return new Pose2d(temPose3d.getX(), temPose3d.getY(), new Rotation2d(temPose3d.getRotation().getX(), temPose3d.getRotation().getY()));
    }

    public GridLocation getSelectedGrid() {
        return getGridLocation(SELECTED_GRID_ID);
    }

    /**
     * checks if two poses are the same
     **/
    public boolean comparePose(Pose2d firstPose, Pose2d secondPose) {
        return ((firstPose.getX() == secondPose.getX()) &&
                (firstPose.getY() == secondPose.getY()) &&
                (firstPose.getRotation().getRadians() == secondPose.getRotation().getRadians()));
    }
}
