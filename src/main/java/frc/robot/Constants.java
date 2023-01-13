// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final String PROJECT_NAME = "Robot2023";

    // Periodic timing of the robot, WPILib default is 0.02 (20ms)
    public static final Double ROBOT_PERIOD = 0.02; // 20 ms

    // Configure the power module used on the robot
    public static final ModuleType POWER_MODULE_TYPE = ModuleType.kRev;

    public static final class RobotDimensions {
        // TODO: replace these with actual dimensions
        public static final double WIDTH = Units.inchesToMeters(0);
        public static final double LENGTH = Units.inchesToMeters(0);

        public static final double WHEEL_CIRCUMFERENCE = Units.inchesToMeters(4) * Math.PI;
        public static final SwerveDriveKinematics SWERVE_DRIVE_KINEMATICS = new SwerveDriveKinematics(
                new Translation2d(-RobotDimensions.LENGTH / 2, RobotDimensions.WIDTH / 2),
                new Translation2d(-RobotDimensions.LENGTH / 2, -RobotDimensions.WIDTH / 2),
                new Translation2d(RobotDimensions.LENGTH / 2, RobotDimensions.WIDTH / 2),
                new Translation2d(RobotDimensions.LENGTH / 2, -RobotDimensions.WIDTH / 2));
    }

    public static final class Drive {
        // Drive settings here
        // TODO: replace these with actual constants
        public static final double MAX_SPEED = 0; // m/s
    }

    public static final class Motor {
        // Motor indexes + configs here
        public static final int ARM_JOINT_INDEX = 0;
        public static final int ARM_JOINT_FOLLOWER_INDEX = 1;
        public static final int ARM_CASCADE_FOLLOWER_INDEX = 2;
        public static final double ARM_JOINT_CURRENT_LIMIT = 20;
        public static final double ARM_CASCADE_CURRENT_LIMIT = 20;
        public static final int ARM_JOINT_ENCODER_RESOLUTION = 4096;
        public static final float ARM_CASCADE_TICKS_PER_FEET = 1200; // PLACEHOLDER
        public static final float ARM_CASCADE_STARTING_HEIGHT = 1.5f;

        public static final int SWERVE_FRONT_LEFT_POWER = 4;
        public static final int SWERVE_FRONT_LEFT_STEER = 15;

        public static final int SWERVE_FRONT_RIGHT_POWER = 3;
        public static final int SWERVE_FRONT_RIGHT_STEER = 14;

        public static final int SWERVE_BACK_LEFT_POWER = 2;
        public static final int SWERVE_BACK_LEFT_STEER = 13;

        public static final int SWERVE_BACK_RIGHT_POWER = 1;
        public static final int SWERVE_BACK_RIGHT_STEER = 12;
    }
}