// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

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

    public static final class DrivetrainConstants {

        public static Alliance alliance = Alliance.Blue;
    }

    public static class ControllerConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;

        public static final double X_DEADBAND = 0;
        public static final double Y_DEADBAND = 0;
        public static final double Z_DEADBAND = 0;
    }

    public static class ShooterConstants {
        public static final int leftMotorId = 15;
        public static final int rightMotorId = 16;
    }

    public static class IntakeConstants {
        public static final double intakeWheelDiameterMeters = Units.inchesToMeters(2.25);
        public static final double intakeWheelCircunferenceMeters = intakeWheelDiameterMeters * Math.PI;

        public static final double[] leftControllerVals = new double[]{0, 0, 0};
        public static final double[] frontControllerVals = new double[]{0, 0, 0};
        public static final double[] rightControllerVals = new double[]{0, 0, 0};

        public static final int leftMotorId = -1;
        public static final int frontMotorId = -1;
        public static final int rightMotorId = -1;
        
        public static final int loaderMotorId = -1;

        public static final int lifter0Channel = 0;
        public static final int lifter1Channel = 1;
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

        // TODO: Actually get measurements, these are currently set as if the camera is
        // in the direct center facing forward with no angle.
        public static final Transform3d ROBOT_TO_LEFT_CAM = new Transform3d(
                new Translation3d(Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(0)),
                new Rotation3d(Math.toRadians(0), Math.toRadians(0), Math.toRadians(0)));
        public static final Transform3d ROBOT_TO_FRONT_CAM = new Transform3d(
                new Translation3d(Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(0)),
                new Rotation3d(Math.toRadians(0), Math.toRadians(0), Math.toRadians(0)));
        public static final Transform3d ROBOT_TO_RIGHT_CAM = new Transform3d(
                new Translation3d(Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(0)),
                new Rotation3d(Math.toRadians(0), Math.toRadians(0), Math.toRadians(0)));
        public static final Transform3d ROBOT_TO_BACK_CAM = new Transform3d(
                new Translation3d(Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(0)),
                new Rotation3d(Math.toRadians(0), Math.toRadians(0), Math.toRadians(0)));

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

    public static class FieldConstants {
        public static final double FIELD_LENGTH_METERS = Units.inchesToMeters(651.25);
        public static final double FIELD_WIDTH_METERS = Units.inchesToMeters(315.5);
    }
}
