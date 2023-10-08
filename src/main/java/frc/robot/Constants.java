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
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
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
                new SwerveDriveKinematics(new Translation2d(RobotDimensions.LENGTH / 2, -RobotDimensions.WIDTH / 2),
                        new Translation2d(RobotDimensions.LENGTH / 2, RobotDimensions.WIDTH / 2),
                        new Translation2d(-RobotDimensions.LENGTH / 2, -RobotDimensions.WIDTH / 2),
                        new Translation2d(-RobotDimensions.LENGTH / 2, RobotDimensions.WIDTH / 2));
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
                temp = AprilTagFieldLayout.loadFromResource("/edu/wpi/first/apriltag/2023-chargedup.json");
            } catch (IOException e) {
                e.printStackTrace();
            }
            TAG_FIELD_LAYOUT = temp;
        }
    }

    public static final class Arm {
        public static final int ARM_LIMIT_SWITCH_PORT = 0;
        public static final int ARM_ABSOLUTE_ENCODER_PORT = 9;
        public static final double ARM_DEGREE_DISTANCE_FROM_ZERO_TO_LIMIT_SWITCH = 36; // PLACEHOLDER degrees // 40mm
        public static final double ARM_ZEROING_SPEED = -20; // deg/sec
        public static final double ARM_ZEROING_VOLTAGE = -2; // volts
        public static final PidProfile ARM_PID_PROFILE = new PidProfile(0.04, 0, 0); // I: .000009
        public static final PidProfile CASCADE_PID_PROFILE = new PidProfile(0.005, 0, 0.001);
        public static final double ARM_JOINT_CURRENT_LIMIT = 20;
        public static final double ARM_CASCADE_CURRENT_LIMIT = 20;
        public static final int ARM_JOINT_ENCODER_RESOLUTION = 4096;
        public static final double ARM_CASCADE_DEG_PER_FOOT = 785.45454; // degrees
        public static final double ARM_CASCADE_STARTING_HEIGHT = 0; // feet
        public static final double ARM_JOINT_OFFSET = 0.75;

        public static final double ARM_POSITION_TOLERANCE = 0.1; // feet
        public static final int ARM_CASCADE_TOLERANCE = 8; // degrees
        public static final double ARM_JOINT_TOLERANCE = 0.8; // degrees
        public static final double ARM_CASCADE_MAX_FEEDFORWARD = 0.65; // volts

        public static final double ARM_PREDICTIVE_TIMESPAN = 0.25; // seconds
        public static final double ARM_HEIGHT_FROM_GROUND = -1.5; // ft
        public static final double ARM_HEIGHT_FROM_BASE = -1; // ft
        public static final double ARM_MIN_ANGLE_COLLISION_A = 0; // degrees
        public static final double ARM_MAX_ANGLE_COLLISION_A = 50; // degrees
        public static final double ARM_MIN_ANGLE_COLLISION_B = 200; // degrees
        public static final double ARM_MAX_ANGLE_COLLISION_B = 280; // degrees

        // Placeholder
        public static final double ARM_PRESET_LOW_ANGLE = 62; // deg, was 60
        public static final double ARM_PRESET_NO_EXTENSION = 0.01; // ft, for new button (??) (anish review)
        public static final double ARM_PRESET_LOW_LENGTH = 1.2; // ft
        public static final double ARM_PRESET_MID_CONE_ANGLE = 118; // deg, was 115
        public static final double ARM_PRESET_MID_CONE_LENGTH = 1.08; // ft, was 1.24
        public static final double ARM_PRESET_MID_CUBE_ANGLE = 106; // deg
        public static final double ARM_PRESET_MID_CUBE_LENGTH = 0.86; // ft
        public static final double ARM_PRESET_HIGH_CONE_ANGLE = 130; // deg
        public static final double ARM_PRESET_HIGH_CONE_LENGTH = 2.2; // ft
        public static final double ARM_PRESET_HIGH_CUBE_ANGLE = 116; // deg, was 112
        public static final double ARM_PRESET_HIGH_CUBE_LENGTH = 2.2; // ft

        public static final double ARM_PRESET_TRAY_ANGLE = 118; // deg
        public static final double ARM_PRESET_TRAY_LENGTH = 1.82; // ft
        public static final double ARM_PRESET_STOW_ANGLE = 174.95; // deg
        public static final double ARM_PRESET_STOW_LENGTH = 0; // ft

        // Geometry
        public static final double ARM_PIVOT_X = Units.inchesToMeters(11);
        public static final double ARM_PIVOT_Y = Units.inchesToMeters(32.5);
        public static final double MIN_ARM_LENGTH = Units.inchesToMeters(18);
    }

    public static final class Claw {
        public static final double CLAW_VELOCITY_IN_CONE = 1500; // RPM
        public static final double CLAW_VELOCITY_IN_CUBE = 350; // RPM, was 300
        public static final double CLAW_VELOCITY_OUT_MID = 150; // RPM
        public static final double CLAW_VELOCITY_OUT_HIGH = 300; // RPMs
        public static final double CLAW_CURRENT_LIMIT = 0.5; // Amps
    }

    public static final class Scoring {

        public static final double AUTO_SWERVE_MAX_VELOCITY = 4; // Meters per second
        public static final double AUTO_SWERVE_MAX_ACCELERATION = 2.5; // Meters per second
        public static final TrajectoryConfig SCORING_TRAJECTORY_CONFIG =
                new TrajectoryConfig(AUTO_SWERVE_MAX_VELOCITY, AUTO_SWERVE_MAX_ACCELERATION);

        // TODO: replace the offsets with actual values
        public static final double MAX_AUTOMOVE_DISTANCE = 2; // meters
        public static final double NEXT_TO_TAG_OFFSET = 0;

        public static final double LEFT_GRID_LEFT_NODE_OFFSET = 0;
        public static final double LEFT_GRID_RIGHT_NODE_OFFSET = 0;
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

        public static final HashMap<Alliance, HashSet<Integer>> TARGETS_PER_ALLIANCE = new HashMap<>();
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
            TARGETS_PER_ALLIANCE.put(Alliance.Blue, blueTargets);
        }

        public static final PIDController X_CONTROLLER = new PIDController(0, 0, 0);
        public static final PIDController Y_CONTROLLER = new PIDController(0, 0, 0);
        public static final ProfiledPIDController THETA_CONTROLLER = new ProfiledPIDController(0, 0, 0,
                new Constraints(AUTO_SWERVE_MAX_VELOCITY, AUTO_SWERVE_MAX_ACCELERATION));
        public static final double TRAJECTORY_SAMPLE_TIME = 0; // seconds

    }

    public static final class Motor {
        // Motor indexes + configs here
        public static final int ARM_JOINT_INDEX = 12;
        public static final int ARM_JOINT_FOLLOWER_INDEX = 13;
        public static final int ARM_CASCADE_INDEX = 11;

        public static final int SWERVE_FRONT_LEFT_POWER = 1;
        public static final int SWERVE_FRONT_LEFT_STEER = 2;

        public static final int SWERVE_FRONT_RIGHT_POWER = 3;
        public static final int SWERVE_FRONT_RIGHT_STEER = 4;

        public static final int SWERVE_BACK_LEFT_POWER = 5;
        public static final int SWERVE_BACK_LEFT_STEER = 6;

        public static final int SWERVE_BACK_RIGHT_POWER = 7;
        public static final int SWERVE_BACK_RIGHT_STEER = 8;

        public static final int ROLLER_LEFT = 14;
        public static final int ROLLER_RIGHT = 15;
    }

    public static final class Pneumatics {
        public static final int CLAW_SOLENOID_CHANNEL = 1;
    }

    public static final class Swerve {
        public static final PidProfile STEER_PROFILE = new PidProfile(0.03, 0, 0.1);
        public static final PidProfile POWER_PROFILE = new PidProfile(0.0002, 0.0, 0);

        public static final double SWERVE_POWER_CURRENT_LIMIT = 60.0;
        public static final double SWERVE_POWER_MAX_OUTPUT = 0.8;

        public static final double SWERVE_ROTATION_CURRENT_LIMIT = 40.0;
        public static final double SWERVE_ROTATION_MAX_OUTPUT = 0.7;

        public static final double DISTANCE_PER_REV = Units.inchesToMeters(4 * Math.PI);
        public static final double NEO_MAX_SPEED = 5600; // RPM
        public static final double MAX_WHEEL_SPEED = ((NEO_MAX_SPEED / 60) * DISTANCE_PER_REV);
        public static final double SWERVE_MAX_SPEED = 6; // m/s
        public static final double SWERVE_MAX_AUTO_SPEED = 0.2 * MAX_WHEEL_SPEED; // m/s
        public static final double SWERVE_MAX_PRECISION_SPEED = 0.1 * MAX_WHEEL_SPEED; // m/s
        public static final double SWERVE_MAX_ACCELERATION = 2; // m/s^2
        public static final double SWERVE_ROTATION_MAX_SPEED = Math.PI * 2; // rad/s
        public static final double SWERVE_ROTATION_MAX_ACCELERATION = Math.PI * 2 / 3; // rads/s^2

        public static final double SWERVE_DEADBAND = 0.05;
        public static final double SWERVE_ROTATION_TOLERANCE = 5; // degrees
        public static final double SWERVE_SNAPPING_DEADBAND = 0.5; // 50%
    }

    public static final class Input {
        // TODO: Fix idx
        public static final ConstantButton SWERVE_FACE_N = new ConstantButton(0, 0);
        public static final ConstantButton SWERVE_FACE_E = new ConstantButton(0, 90);
        public static final ConstantButton SWERVE_FACE_S = new ConstantButton(0, 180);
        public static final ConstantButton SWERVE_FACE_W = new ConstantButton(0, 270);

        public static final ConstantButton SWERVE_FACE_ALLIANCE = new ConstantButton(0, 6);
        public static final ConstantButton SWERVE_FACE_HUMAN_PLAYER = new ConstantButton(0, 5);

        public static final ConstantAxis SWERVE_SNAP_ROTATION_X = new ConstantAxis(0, 2);
        public static final ConstantAxis SWERVE_SNAP_ROTATION_Y = new ConstantAxis(0, 3);
        public static final ConstantAxis SWERVE_X_INPUT = new ConstantAxis(0, 0);
        public static final ConstantAxis SWERVE_Y_INPUT = new ConstantAxis(0, 1);
        public static final ConstantAxis SWERVE_ROTATION_INPUT = new ConstantAxis(0, 4);
        public static final ConstantAxis SWERVE_ROTATION_SLOWDOWN_L  = new ConstantAxis(0, 2);
        public static final ConstantAxis SWERVE_ROTATION_SLOWDOWN_R = new ConstantAxis(0, 3);

        public static final ConstantAxis ARM_MANUAL_MOVEMENT_UP_DOWN = new ConstantAxis(2, 1);
        public static final ConstantAxis ARM_MANUAL_MOVEMENT_FORWARD_BACKWARD = new ConstantAxis(2, 0);
        // scoring
        // public static final ConstantButton POSITION_TO_CLOSEST_GRID = new ConstantButton(1, 10);
        // public static final ConstantButton POSITION_TO_HUMAN_PLAYER = new ConstantButton(1, 11);

        // public static final ConstantButton UPPER_LEFT_CONE = new ConstantButton(1, 1);
        // public static final ConstantButton UPPER_MIDDLE_CUBE = new ConstantButton(1, 2);
        // public static final ConstantButton UPPER_RIGHT_CONE = new ConstantButton(1, 3);
        // public static final ConstantButton MIDDLE_LEFT_CONE = new ConstantButton(1, 4);
        // public static final ConstantButton MIDDLE_MIDDLE_CUBE = new ConstantButton(1, 5);
        // public static final ConstantButton MIDDLE_RIGHT_CONE = new ConstantButton(1, 6);
        // public static final ConstantButton HYBRID_LEFT = new ConstantButton(1, 7);
        // public static final ConstantButton HYBRID_MIDDLE = new ConstantButton(1, 8);
        // public static final ConstantButton HYBRID_RIGHT = new ConstantButton(1, 9);

        public static final ConstantButton CLAW_IN_CONE = new ConstantButton(3, 11);
        public static final ConstantButton CLAW_IN_CUBE = new ConstantButton(3, 9);
        public static final ConstantButton CLAW_OUT_MID = new ConstantButton(3, 3);
        public static final ConstantButton CLAW_OUT_HIGH = new ConstantButton(3, 2);
        // public static final ConstantButton TEST_A = new ConstantButton(0, 1);
        // public static final ConstantButton TEST_B = new ConstantButton(0, 2);
        // public static final ConstantButton LED_TRIGGER_PURPLE = new ConstantButton(0, 3);
        // public static final ConstantButton LED_TRIGGER_YELLOW = new ConstantButton(0, 4);
        public static final ConstantButton MID_CUBE_SCORE = new ConstantButton(3, 8);
        public static final ConstantButton MID_CONE_SCORE = new ConstantButton(3, 10);
        public static final ConstantButton HIGH_CUBE_SCORE = new ConstantButton(3, 5);
        public static final ConstantButton HIGH_CONE_SCORE = new ConstantButton(3, 6);
        public static final ConstantButton LOW_SCORE = new ConstantButton(3, 7); // was 4, this is supposed to be ground intake (aka old hybrid)
        public static final ConstantButton GROUND_INTAKE = new ConstantButton(3, 4); // was 7, this is the new thing we coded without extension
        public static final ConstantButton STOW = new ConstantButton(3, 12);
        public static final ConstantButton TRAY = new ConstantButton(3, 1);
    }
    public static final class Auto {
        private static final double SWERVE_AUTO_SPEED_MULTIPLIER = 0.6;
        public static final TrapezoidProfile.Constraints SWERVE_STRAFE_PID_CONSTRAINTS =
                new TrapezoidProfile.Constraints(Swerve.SWERVE_MAX_SPEED * SWERVE_AUTO_SPEED_MULTIPLIER,
                        Swerve.SWERVE_MAX_ACCELERATION);
        public static final TrapezoidProfile.Constraints SWERVE_ROTATION_PID_CONSTRAINTS =
                new TrapezoidProfile.Constraints(36000, 36000);
        public static final ProfiledPIDController PROFILED_ROT_PID_CONTROLLER =
                new ProfiledPIDController(10, 0, 0, Constants.Auto.SWERVE_ROTATION_PID_CONSTRAINTS);
        public static final PIDController X_PID_CONTROLLER = new PIDController(0, 0, 0);
        public static final PIDController Y_PID_CONTROLLER = new PIDController(0, 0, 0);
        public static final PIDController ROT_PID_CONTROLLER = new PIDController(0, 0, 0);
    }
    public static final class PoseEstimation {
        public static final Matrix<N3, N1> POSE_GYRO_STD = VecBuilder.fill(0.1, 0.1, 0.1);
        public static final Matrix<N3, N1> POSE_VISION_STD = VecBuilder.fill(0.1, 0.1, 0.1);
    }
}
