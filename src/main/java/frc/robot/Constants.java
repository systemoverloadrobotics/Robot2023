// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import frc.sorutil.ConstantAxis;
import frc.sorutil.motor.PidProfile;

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
        public static final double WIDTH = Units.inchesToMeters(28);
        public static final double LENGTH = Units.inchesToMeters(26);

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

    public static final class Vision {
        // Camera location from the center of the robot
        public static final double CAMERA_POSITION_X = 0; // Meters
        public static final double CAMERA_POSITION_Y = 0.5; // Meters
        public static final double CAMERA_POSITION_Z = 0; // Meters
        public static final double CAMERA_ROTATION_ROLL = 0; // Radians
        public static final double CAMERA_ROTATION_PITCH = 0; // Radians
        public static final double CAMERA_ROTATION_YAW = 0; // Radians

        public static final Translation3d CAMERA_POSITION = new Translation3d(CAMERA_POSITION_X, CAMERA_POSITION_Y, CAMERA_POSITION_Z);
        public static final Rotation3d CAMERA_ROTATION = new Rotation3d(CAMERA_ROTATION_ROLL,CAMERA_ROTATION_PITCH,CAMERA_ROTATION_YAW);
    }

    public static final class Motor {
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
    }
    
    public static final class Pneumatics {
        public static final int CLAW_SOLENOID_CHANNEL = 1;
    }

    public static final class Swerve {
        public static final PidProfile STEER_PROFILE = new PidProfile(0.03, 0, 0.1);
        public static final PidProfile POWER_PROFILE = new PidProfile(0.0001, 0.0, 0);
  
        public static final double SWERVE_MAX_SPEED = 5.18; // m/s
        public static final double SWERVE_MAX_ACCELERATION = 3; // m/s^2
        public static final double SWERVE_ROTATION_MAX_SPEED = 3; // rad/s
        public static final double SWERVE_ROTATION_MAX_ACCELERATION = Math.PI; // rads/s^2
  
        public static final double SWERVE_DEADBAND = 0.05;
  
        public static final class Characteristics {
          public static final double MPS_TO_RPM = 1315 / 4;
          public static final double COUNTS_PER_100MS = 4201;
        }
      }
      public static final class Input {
        public static final ConstantAxis SWERVE_X = new ConstantAxis(0, 1);
        public static final ConstantAxis SWERVE_Y = new ConstantAxis(0, 2);
        public static final ConstantAxis SWERVE_ROTATION = new ConstantAxis(1, 1);
      }
}