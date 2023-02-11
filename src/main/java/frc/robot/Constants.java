// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.util.HashMap;
import java.util.HashSet;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import frc.sorutil.ConstantAxis;
import frc.sorutil.ConstantButton;
import frc.sorutil.motor.PidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes) wherever the constants are needed, to
 * reduce verbosity.
 */
public final class Constants {
    public static final String PROJECT_NAME = "Robot2023";

    // Periodic timing of the robot, WPILib default is 0.02 (20ms)
    public static final Double ROBOT_PERIOD = 0.02; // 20 ms

    // Configure the power module used on the robot
    public static final ModuleType POWER_MODULE_TYPE = ModuleType.kRev;

    public static final double NOMINAL_VOLTAGE = 12.0;

    public static final class RobotDimensions {
        // TODO: replace these with actual dimensions
        public static final double WIDTH = Units.inchesToMeters(28);
        public static final double LENGTH = Units.inchesToMeters(26);

        public static final double WHEEL_CIRCUMFERENCE = Units.inchesToMeters(4) * Math.PI;
        public static final SwerveDriveKinematics SWERVE_DRIVE_KINEMATICS =
                new SwerveDriveKinematics(
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

    public static final class Vision {
        // Camera location from the center of the robot
        public static final double CAMERA_POSITION_X = 0; // Meters
        public static final double CAMERA_POSITION_Y = 0.5; // Meters
        public static final double CAMERA_POSITION_Z = 0; // Meters
        public static final double CAMERA_ROTATION_ROLL = 0; // Radians
        public static final double CAMERA_ROTATION_PITCH = 0; // Radians
        public static final double CAMERA_ROTATION_YAW = 0; // Radians

        public static final Translation3d CAMERA_POSITION =
                new Translation3d(CAMERA_POSITION_X, CAMERA_POSITION_Y, CAMERA_POSITION_Z);
        public static final Rotation3d CAMERA_ROTATION =
                new Rotation3d(CAMERA_ROTATION_ROLL, CAMERA_ROTATION_PITCH, CAMERA_ROTATION_YAW);

        public static final AprilTagFieldLayout TAG_FIELD_LAYOUT;

        static {
            AprilTagFieldLayout temp = null;
            try {
                temp = AprilTagFieldLayout
                        .loadFromResource("/edu/wpi/first/apriltag/2023-chargedup.json");
            } catch (IOException e) {
                e.printStackTrace();
            }
            TAG_FIELD_LAYOUT = temp;
        }
    }

    public static final class Arm {
        public static final PidProfile ARM_PID_PROFILE = new PidProfile(0, 0, 0);
        public static final PidProfile CASCADE_PID_PROFILE = new PidProfile(0, 0, 0);
        public static final double ARM_JOINT_CURRENT_LIMIT = 20;
        public static final double ARM_CASCADE_CURRENT_LIMIT = 20;
        public static final int ARM_JOINT_ENCODER_RESOLUTION = 4096;
        public static final double ARM_CASCADE_TICKS_PER_FEET = 1200; // PLACEHOLDER
        public static final double ARM_CASCADE_STARTING_HEIGHT = 1.5; // feet
        public static final int ARM_CASCADE_TOLERANCE = 8; // units
        public static final int ARM_JOINT_TOLERANCE = 8; // units
        public static final double ARM_PREDICTIVE_TIMESPAN = 0.25; // seconds
        public static final double ARM_HEIGHT_FROM_GROUND = -1.5; // ft
        public static final double ARM_HEIGHT_FROM_BASE = -1; // ft
        public static final double ARM_MIN_ANGLE_COLLISION_A = 300; // degrees
        public static final double ARM_MAX_ANGLE_COLLISION_A = 330; // degrees
        public static final double ARM_MIN_ANGLE_COLLISION_B = 200; // degrees
        public static final double ARM_MAX_ANGLE_COLLISION_B = 280; // degrees

        // Placeholder
        public static final double ARM_PRESET_LOW_X = 3; // ft
        public static final double ARM_PRESET_LOW_Y = 1; // ft
        public static final double ARM_PRESET_MID_CONE_X = 3; // ft
        public static final double ARM_PRESET_MID_CONE_Y = 2; // ft
        public static final double ARM_PRESET_MID_CUBE_X = 3; // ft
        public static final double ARM_PRESET_MID_CUBE_Y = 2; // ft
        public static final double ARM_PRESET_HIGH_CONE_X = 3; // ft
        public static final double ARM_PRESET_HIGH_CONE_Y = 3; // ft
        public static final double ARM_PRESET_HIGH_CUBE_X = 3; // ft
        public static final double ARM_PRESET_HIGH_CUBE_Y = 3; // ft
        public static final double ARM_PRESET_TRAY_X = 3; // ft
        public static final double ARM_PRESET_TRAY_Y = 2; // ft
        public static final double ARM_PRESET_STOW_X = 0; // ft
        public static final double ARM_PRESET_STOW_Y = 1; // ft

        // Geometry
        public static final double ARM_PIVOT_X = Units.inchesToMeters(11);
        public static final double ARM_PIVOT_Y = Units.inchesToMeters(32.5);
        public static final double MIN_ARM_LENGTH = Units.inchesToMeters(18);

        // Preset Heights
    }

    public static final class Claw {
        public static final double CLAW_VELOCITY = 1000; // units/sec
    }

    public static final class Scoring {

        public static final double AUTO_SWERVE_MAX_VELOCITY = 3; // Meters per second
        public static final double AUTO_SWERVE_MAX_ACCELERATION = 2.5; // Meters per second
        public static final TrajectoryConfig SCORING_TRAJECTORY_CONFIG =
                new TrajectoryConfig(AUTO_SWERVE_MAX_VELOCITY, AUTO_SWERVE_MAX_ACCELERATION);

        //TODO: replace the offsets with actual values
        public static final double MIN_AUTOMOVE_DISTANCE = 2; // meters
        public static final double NEXT_TO_TAG_OFFSET = 0; 
        
        public static final double LEFT_GRID_LEFT_NODE_OFFSET = 0; 
        public static final double LEFT_GRID_RIGHT_NODE_OFFSET = 0; 
        public static final double MIDDLE_GRID_LEFT_NODE_OFFSET = 0; 
        public static final double MIDDLE_GRID_RIGHT_NODE_OFFSET = 0; 
        public static final double RIGHT_GRID_LEFT_NODE_OFFSET = 0; 
        public static final double RIGHT_GRID_RIGHT_NODE_OFFSET = 0;
        

        public static final HashMap<Integer, Alliance> VALID_SCORING_TARGETS = new HashMap<>();
        static {
            VALID_SCORING_TARGETS.put(1, Alliance.Red);
            VALID_SCORING_TARGETS.put(2, Alliance.Red);
            VALID_SCORING_TARGETS.put(3, Alliance.Red);
            VALID_SCORING_TARGETS.put(6, Alliance.Blue);
            VALID_SCORING_TARGETS.put(7, Alliance.Blue);
            VALID_SCORING_TARGETS.put(8, Alliance.Blue);
        }

        public static final HashMap<Alliance, HashSet<Integer>> TARGETS_PER_ALLIANCE =
                new HashMap<>();
        static {
            var redTargets = new HashSet<Integer>();
            redTargets.add(1);
            redTargets.add(2);
            redTargets.add(3);
            var blueTargets = new HashSet<Integer>();
            blueTargets.add(6);
            blueTargets.add(7);
            blueTargets.add(8);
            TARGETS_PER_ALLIANCE.put(Alliance.Red, redTargets);
            TARGETS_PER_ALLIANCE.put(Alliance.Red, blueTargets);
        }

        public static final PIDController X_CONTROLLER = new PIDController(0, 0, 0);
        public static final PIDController Y_CONTROLLER = new PIDController(0, 0, 0);
        public static final ProfiledPIDController THETA_CONTROLLER = new ProfiledPIDController(0, 0,
                0, new Constraints(AUTO_SWERVE_MAX_VELOCITY, AUTO_SWERVE_MAX_ACCELERATION));
        public static final double TRAJECTORY_SAMPLE_TIME = 0; // seconds
    }

    public static final class Motor {
        // Motor indexes + configs here
        public static final int ARM_JOINT_INDEX = 0;
        public static final int ARM_JOINT_FOLLOWER_INDEX = 1;
        public static final int ARM_CASCADE_INDEX = 20;

        public static final int SWERVE_FRONT_LEFT_POWER = 1;
        public static final int SWERVE_FRONT_LEFT_STEER = 2;

        public static final int SWERVE_FRONT_RIGHT_POWER = 3;
        public static final int SWERVE_FRONT_RIGHT_STEER = 4;

        public static final int SWERVE_BACK_LEFT_POWER = 5;
        public static final int SWERVE_BACK_LEFT_STEER = 6;

        public static final int SWERVE_BACK_RIGHT_POWER = 7;
        public static final int SWERVE_BACK_RIGHT_STEER = 8;

        public static final int ROLLER_LEFT = 5;
        public static final int ROLLER_RIGHT = 6;

        public static final double CLAW_VOLTAGE = 0;
    }

    public static final class Pneumatics {
        public static final int CLAW_SOLENOID_CHANNEL = 1;
    }

    public static final class Swerve {
        public static final PidProfile STEER_PROFILE = new PidProfile(0.03, 0, 0.1);
        public static final PidProfile POWER_PROFILE = new PidProfile(0.0001, 0.0, 0);

        public static final double SWERVE_POWER_CURRENT_LIMIT = 15.0;
        public static final double SWERVE_POWER_MAX_OUTPUT = 0.5;

        public static final double SWERVE_ROTATION_CURRENT_LIMIT = 15.0;
        public static final double SWERVE_ROTATION_MAX_OUTPUT = 0.5;

        public static final double DISTANCE_PER_REV = Units.inchesToMeters(4 * Math.PI);
        public static final double NEO_MAX_SPEED = 5600; // RPM
        public static final double MAX_WHEEL_SPEED =
                ((NEO_MAX_SPEED / 60) * DISTANCE_PER_REV) / 6.75;
        public static final double SWERVE_MAX_SPEED = 0.9 * MAX_WHEEL_SPEED; // m/s
        public static final double SWERVE_MAX_ACCELERATION = 3; // m/s^2
        public static final double SWERVE_ROTATION_MAX_SPEED = 3; // rad/s
        public static final double SWERVE_ROTATION_MAX_ACCELERATION = Math.PI; // rads/s^2

        public static final double SWERVE_DEADBAND = 0.05;
    }

    public static final class Input {
        public static final ConstantAxis SWERVE_X_INPUT = new ConstantAxis(1, 0);
        public static final ConstantAxis SWERVE_Y_INPUT = new ConstantAxis(1, 1);
        public static final ConstantAxis SWERVE_ROTATION_INPUT = new ConstantAxis(0, 0);

        //scoring
        public static final ConstantButton UPPER_LEFT_CONE = new ConstantButton(1, 0);
        public static final ConstantButton UPPER_MIDDLE_CUBE = new ConstantButton(1, 1);
        public static final ConstantButton UPPER_RIGHT_CONE = new ConstantButton(1, 2);
        public static final ConstantButton MIDDLE_LEFT_CONE = new ConstantButton(1, 3);
        public static final ConstantButton MIDDLE_MIDDLE_CUBE = new ConstantButton(1, 4);
        public static final ConstantButton MIDDLE_RIGHT_CONE = new ConstantButton(1, 5);
        public static final ConstantButton HYBRID_LEFT = new ConstantButton(1, 6);
        public static final ConstantButton HYBRID_MIDDLE = new ConstantButton(1, 7);
        public static final ConstantButton HYBRID_RIGHT = new ConstantButton(1, 8);
    }
}
