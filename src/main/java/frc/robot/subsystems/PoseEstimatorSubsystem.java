package frc.robot.subsystems;

import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;

import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;

public class PoseEstimatorSubsystem extends SubsystemBase {
    // private final Supplier<Rotation2d> rotationSupplier;
    // private final Supplier<SwerveModulePosition[]> modulePositionSupplier;
    // private final SwerveDrivePoseEstimator poseEstimator;
    private final SwerveSubsystem swerveSubsystem;
    private final PhotonRunnable leftEstimator = new PhotonRunnable(new PhotonCamera("LeftCam"),
            VisionConstants.ROBOT_TO_LEFT_CAM);
    // private final PhotonRunnable frontEstimator = new PhotonRunnable(new PhotonCamera("FisheyeCam"), // May not use this for positioning
    //         VisionConstants.ROBOT_TO_FRONT_CAM);
    private final PhotonRunnable rightEstimator = new PhotonRunnable(new PhotonCamera("RightCam"),
            VisionConstants.ROBOT_TO_RIGHT_CAM);
    private final PhotonRunnable backEstimator = new PhotonRunnable(new PhotonCamera("BackCam"),
            VisionConstants.ROBOT_TO_BACK_CAM);

    private final Notifier allNotifier = new Notifier(() -> {
        // frontEstimator.run();
        leftEstimator.run();
        backEstimator.run();
        rightEstimator.run();
    });

    private OriginPosition originPosition = OriginPosition.kBlueAllianceWallRightSide;

    public PoseEstimatorSubsystem(Supplier<Rotation2d> rotationSupplier,
            Supplier<SwerveModulePosition[]> modulePositionSupplier, SwerveSubsystem swerveSubsystem) {
        // this.rotationSupplier = rotationSupplier;
        // this.modulePositionSupplier = modulePositionSupplier;
        this.swerveSubsystem = swerveSubsystem;

        allNotifier.setName("runAllVision");
        allNotifier.startPeriodic(0.02);
    }

    @Override
    public void periodic() {
        if (VisionConstants.USE_VISION) {
            // estimatorChecker(frontEstimator);
            estimatorChecker(leftEstimator);
            estimatorChecker(backEstimator);
            estimatorChecker(rightEstimator);
        } else {
            allNotifier.close();
        }
    }

    // private String getFomattedPose() {
    //     var pose = getCurrentPose();
    //     return String.format("(%.3f, %.3f) %.2f degrees",
    //             pose.getX(),
    //             pose.getY(),
    //             pose.getRotation().getDegrees());
    // }

    /**
     * Sets the alliance. This is used to configure the origin of the AprilTag map
     * 
     * @param alliance alliance
     */
    public void setAlliance(Alliance alliance) {
        boolean allianceChanged = false;
        switch (alliance) {
            case Blue:
                allianceChanged = (originPosition == OriginPosition.kRedAllianceWallRightSide);
                originPosition = OriginPosition.kBlueAllianceWallRightSide;
                break;
            case Red:
                allianceChanged = (originPosition == OriginPosition.kBlueAllianceWallRightSide);
                originPosition = OriginPosition.kRedAllianceWallRightSide;
                break;
            default:
                // No valid alliance data. Nothing we can do about it
        }

        if (allianceChanged) {
            // The alliance changed, which changes the coordinate system.
            // Since a tag was seen, and the tags are all relative to the coordinate system,
            // the estimated pose
            // needs to be transformed to the new coordinate system.
            Pose2d newPose = flipAlliance(getCurrentPose());
            swerveSubsystem.getSwerveDrive().resetOdometry(newPose);
        }
    }

    public Pose2d getCurrentPose() {
        return swerveSubsystem.getPose();
    }

    /**
     * Resets the current pose to the specified pose. This should ONLY be called
     * when the robot's position on the field is known, like at the beginning of
     * a match.
     * 
     * @param newPose new pose
     */
    public void setCurrentPose(Pose2d newPose) {
        swerveSubsystem.getSwerveDrive().resetOdometry(newPose);
    }

    /**
     * Resets the position on the field to 0,0 0-degrees, with forward being
     * downfield. This resets
     * what "forward" is for field oriented driving.
     */
    public void resetFieldPosition() {
        setCurrentPose(new Pose2d());
    }

    /**
     * Transforms a pose to the opposite alliance's coordinate system. (0,0) is
     * always on the right corner of your
     * alliance wall
     * 
     * @param poseToFlip pose to transform to the other alliance
     * @return pose relative to the other alliance's coordinate system
     */
    private Pose2d flipAlliance(Pose2d poseToFlip) {
        return poseToFlip.relativeTo(new Pose2d(
                new Translation2d(FieldConstants.FIELD_LENGTH_METERS, FieldConstants.FIELD_WIDTH_METERS),
                new Rotation2d(Math.PI)));
    }

    private Matrix<N3, N1> confidenceCalculator(EstimatedRobotPose estimation) {
        double smallestDistance = Double.POSITIVE_INFINITY;
        for (var target : estimation.targetsUsed) {
            var t3d = target.getBestCameraToTarget();
            var distance = Math.sqrt(Math.pow(t3d.getX(), 2) + Math.pow(t3d.getY(), 2) + Math.pow(t3d.getZ(), 2));
            if (distance < smallestDistance)
                smallestDistance = distance;
        }
        double poseAmbiguityFactor = estimation.targetsUsed.size() != 1
                ? 1
                : Math.max(
                        1,
                        (estimation.targetsUsed.get(0).getPoseAmbiguity()
                                + Constants.VisionConstants.POSE_AMBIGUITY_SHIFTER)
                                * Constants.VisionConstants.POSE_AMBIGUITY_MULTIPLIER);
        double confidenceMultiplier = Math.max(
                1,
                (Math.max(
                        1,
                        Math.max(0, smallestDistance - Constants.VisionConstants.NOISY_DISTANCE_METERS)
                                * Constants.VisionConstants.DISTANCE_WEIGHT)
                        * poseAmbiguityFactor)
                        / (1
                                + ((estimation.targetsUsed.size() - 1)
                                        * Constants.VisionConstants.TAG_PRESENCE_WEIGHT)));

        return Constants.VisionConstants.VISION_MEASUREMENT_STANDARD_DEVIATIONS.times(confidenceMultiplier);
    }

    public void estimatorChecker(PhotonRunnable estamator) {
        var cameraPose = estamator.grabLatestEstimatedPose();
        if (cameraPose != null) {
            // New pose from vision
            var pose2d = cameraPose.estimatedPose.toPose2d();
            if (originPosition == OriginPosition.kRedAllianceWallRightSide) {
                pose2d = flipAlliance(pose2d);
            }
            swerveSubsystem.getSwerveDrive().addVisionMeasurement(pose2d, cameraPose.timestampSeconds,
                    confidenceCalculator(cameraPose));
        }
    }
}
