package frc.robot.subsystems;

import java.util.concurrent.atomic.AtomicReference;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants.FieldConstants;
import static frc.robot.Constants.VisionConstants.APRILTAG_AMBIGUITY_THRESHOLD;

public class PhotonRunnable implements Runnable {

    private final PhotonPoseEstimator photonPoseEstimator;
    private final PhotonCamera photonCamera;
    private final AtomicReference<EstimatedRobotPose> atomicEstimatedRobotPose = new AtomicReference<EstimatedRobotPose>();

    public PhotonRunnable(PhotonCamera camera, Transform3d robotToCamera) {
        this.photonCamera = camera;
        PhotonPoseEstimator poseEstimator = null;
        AprilTagFieldLayout layout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        // PV estimates will always be blue, they'll get flipped by robot thread
        layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
        if (this.photonCamera != null) {
            poseEstimator = new PhotonPoseEstimator(layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                    robotToCamera);
            poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        }
        this.photonPoseEstimator = poseEstimator;
    }

    @Override
    public void run() {
        // Get AprilTag data
        if (this.photonPoseEstimator != null && this.photonCamera != null) {
            PhotonPipelineResult photonResults = photonCamera.getLatestResult();
            if (photonResults.hasTargets() && (photonResults.targets.size() > 1
                    || photonResults.targets.get(0).getPoseAmbiguity() < APRILTAG_AMBIGUITY_THRESHOLD)) {
                this.photonPoseEstimator.update(photonResults).ifPresent(estimatedRobotPose -> {
                    Pose3d estimatedPose = estimatedRobotPose.estimatedPose;
                    if (estimatedPose.getX() > 0.0 && estimatedPose.getX() <= FieldConstants.FIELD_LENGTH_METERS
                            && estimatedPose.getY() > 0.0
                            && estimatedPose.getY() <= FieldConstants.FIELD_WIDTH_METERS) {
                        atomicEstimatedRobotPose.set(estimatedRobotPose);
                    }
                });
            }
        }
    }

    public EstimatedRobotPose grabLatestEstimatedPose() {
        return atomicEstimatedRobotPose.getAndSet(null);
    }

}
