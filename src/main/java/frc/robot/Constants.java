// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    // Periodic timing of the robot, WPILib default is 0.02 (20ms)
    public static final Double ROBOT_PERIOD = 0.02; // 20 ms

    public static final class RobotDimensions {
        // TODO: replace these with actual dimensions
        public static final double WIDTH = Units.inchesToMeters(0);
        public static final double LENGTH = Units.inchesToMeters(0);

        public static final double WHEEL_CIRCUMFERENCE = Units.inchesToMeters(4) * Math.PI;
    }

    public static final class Drive {
        // Drive settings here
        // TODO: replace these with actual constants
        public static final double MAX_SPEED = 0; // m/s
    }

    public static final class Motor {
        // Motor indexes here
    }
}
