// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import swervelib.math.Matter;

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

    public static final double ROBOT_MASS = ((100 /* robot */) + (12.89 + 15 /* battery & bumper */)) / 2.2; // lb /2.2
                                                                                                             // = kg
    public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(6)), ROBOT_MASS);
    public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag

    public static class FieldConstants {
        public static final double FIELD_LENGTH_METERS = Units.inchesToMeters(651.25);
        public static final double FIELD_WIDTH_METERS = Units.inchesToMeters(315.5);

        public static final Translation2d BLUE_SPEAKER = new Translation2d(0, 6.0);
        public static final Translation2d RED_SPEAKER = new Translation2d(17.42, 5.98);
    }

    public static final class DrivetrainConstants {

        public static Alliance alliance = Alliance.Blue;
        public static PIDConstants autonomousDriveAim = new PIDConstants(0.4, 0, 0.02);
    }

    public static class ControllerConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;

        public static final double X_DEADBAND = 0.1;
        public static final double Y_DEADBAND = 0.1;
        public static final double Z_DEADBAND = 0.2;
    }

    public static class ShooterConstants {
        public static final int leftMotorId = 31;
        public static final int rightMotorId = 32;
        public static final int aimMotorId = 33;

        public static final int aimPotentiometer = 0;
        public static final double[] aimControllerVals = new double[] { 0.7, 0.002, 0.07 }; // P, I, D
    }

    public static class ClimberConstants {
        public static final int climbMotorId = 20;
    }

    public static class IntakeConstants {
        public static final double intakeWheelDiameterMeters = Units.inchesToMeters(2.25);
        public static final double intakeWheelCircunferenceMeters = intakeWheelDiameterMeters * Math.PI;

        public static final double[] leftControllerVals = new double[] { 0, 0, 0 }; // P, I, D
        public static final double[] frontControllerVals = new double[] { 0, 0, 0 }; // P, I, D
        public static final double[] rightControllerVals = new double[] { 0, 0, 0 }; // P, I, D

        public static final int leftMotorId = 21;
        public static final int frontMotorId = 22;
        public static final int rightMotorId = 23;

        public static final int loaderMotorId = 30;

        public static final int lifterOne = 25;
        public static final int lifterTwo = 26;
        public static final int lifterThree = 27;
    }

    public static class VisionConstants {
        public static boolean USE_VISION = true;
        /** Minimum target ambiguity. Targets with higher ambiguity will be discarded */
        public static final double APRILTAG_AMBIGUITY_THRESHOLD = 0.2;
        public static final double POSE_AMBIGUITY_SHIFTER = 0.2;
        public static final double POSE_AMBIGUITY_MULTIPLIER = 4;
        public static final double NOISY_DISTANCE_METERS = 2.5;
        public static final double DISTANCE_WEIGHT = 7;
        public static final int TAG_PRESENCE_WEIGHT = 10;

        public static final Transform3d ROBOT_TO_LEFT_CAM = new Transform3d(
                new Translation3d(Units.inchesToMeters(-3.25), Units.inchesToMeters(11.25),
                        Units.inchesToMeters(17.75)),
                new Rotation3d(Math.toRadians(0), Math.toRadians(-10), Math.toRadians(-90)));
        // Translation is Forward, Left, Up positive
        // Rotation is Roll, Pitch, Yaw (ccw positive)
        // TODO: Get measurements
        // public static final Transform3d ROBOT_TO_FRONT_CAM = new Transform3d(
        // new Translation3d(Units.inchesToMeters(0), Units.inchesToMeters(0),
        // Units.inchesToMeters(0)),
        // new Rotation3d(Math.toRadians(0), Math.toRadians(0), Math.toRadians(0)));

        public static final Transform3d ROBOT_TO_RIGHT_CAM = new Transform3d(
                new Translation3d(Units.inchesToMeters(-3.25), Units.inchesToMeters(-11.25),
                        Units.inchesToMeters(17.75)),
                new Rotation3d(Math.toRadians(0), Math.toRadians(-10), Math.toRadians(90)));

        // public static final Transform3d ROBOT_TO_BACK_CAM = new Transform3d(
        // new Translation3d(Units.inchesToMeters(-17.5), Units.inchesToMeters(-8.75),
        // Units.inchesToMeters(9.75)),
        // new Rotation3d(Math.toRadians(0), Math.toRadians(-10), Math.toRadians(180)));

        /**
         * Standard deviations of model states. Increase these numbers to trust your
         * model's state estimates less. This
         * matrix is in the form [x, y, theta]ᵀ, with units in meters and radians, then
         * meters.
         */
        public static final Matrix<N3, N1> VISION_MEASUREMENT_STANDARD_DEVIATIONS = Matrix.mat(Nat.N3(), Nat.N1())
                .fill(
                        // if these numbers are less than one, multiplying will do bad things
                        1, // x
                        1, // y
                        1 * Math.PI // theta
                );

        /**
         * Standard deviations of the vision measurements. Increase these numbers to
         * trust global measurements from vision
         * less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and
         * radians.
         */
        public static final Matrix<N3, N1> STATE_STANDARD_DEVIATIONS = Matrix.mat(Nat.N3(), Nat.N1())
                .fill(
                        // if these numbers are less than one, multiplying will do bad things
                        .1, // x
                        .1, // y
                        .1);
    }
}
