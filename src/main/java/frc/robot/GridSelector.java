package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.DriveTrainPoseEstimator;
import frc.robot.subsystems.Vision;

public class GridSelector {
    // gives closest apriltag to the bot
    public static int getClosestId(Vision vision, DriveTrainPoseEstimator poseEstimator) {
        Pose2d currentPose = poseEstimator.getEstimatedPose();

        var allianceTargets =
                Constants.Scoring.TARGETS_PER_ALLIANCE.get(DriverStation.getAlliance());
        double closestDistance = -1;
        int closestId = 0;
        // finds the closest april tag to the bot
        for (int id : allianceTargets) {
            double tempIdDistance = distanceFormula(currentPose.getX(), currentPose.getY(),
                    getTagPose2d(id).getX(), getTagPose2d(id).getY());
            if (tempIdDistance > closestDistance) {
                closestDistance = tempIdDistance;
                closestId = id;
            }
        }
        // checks if the closest april tag if within the minimum distance
        Pose2d targetTagPose = getTagPose2d(closestId);
        if (distanceFormula(currentPose.getX(), currentPose.getY(), targetTagPose.getX(), targetTagPose.getY()) > Constants.Scoring.MIN_AUTOMOVE_DISTANCE) {
            return -1;
        }
        return closestId;
    }
    
    public static GridLocation getGridLocation(int id) {
        switch (id) {
            case 1:
                return GridLocation.RIGHT;
            case 2:
                return GridLocation.MIDDLE;
            case 3:
                return GridLocation.LEFT;
            case 6:
                return GridLocation.RIGHT;
            case 7:
                return GridLocation.MIDDLE;
            case 8:
                return GridLocation.LEFT;
            default:
                return null;
        }
    }

    public static enum GridLocation {
        LEFT, MIDDLE, RIGHT
    }

    public static Pose2d getTagPose2d(int id) {
        Pose3d temPose3d = (Constants.Vision.TAG_FIELD_LAYOUT.getTagPose(id).get());
        return new Pose2d(temPose3d.getX(), temPose3d.getY(), new Rotation2d(temPose3d.getRotation().getX(), temPose3d.getRotation().getY()));
    }

    private static double distanceFormula(double x1, double y1, double x2, double y2) {
        return Math.sqrt((Math.pow(y2 - y1, 2)) + (Math.pow(x2 - x1, 2)));
    }


}
