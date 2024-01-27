
// package frc.lib.drivers.util.vision;

// import edu.wpi.first.apriltag.AprilTagFieldLayout;
// import edu.wpi.first.apriltag.AprilTagFields;
// import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
// import edu.wpi.first.math.Vector;
// import edu.wpi.first.math.geometry.Transform3d;
// import edu.wpi.first.math.numbers.N3;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import frc.robot.Constants;

// import java.io.IOException;
// import java.util.Optional;

// import org.photonvision.PhotonCamera;
// import org.photonvision.PhotonPoseEstimator;
// import org.photonvision.PhotonPoseEstimator.PoseStrategy;

// //From FRC Team 3636 Generals
// public class PhotonVisionBackend extends VisionBackend {
//     private final PhotonCamera camera;
//     private final PhotonPoseEstimator poseEstimator;

//     public PhotonVisionBackend(String name, Transform3d camera_transform) throws IOException {
//         camera = new PhotonCamera(name);
//         camera.setDriverMode(false);
//         camera.setPipelineIndex(0);

//         AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);

//         poseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, camera_transform);
//     }

//     @Override
//     public Optional<Measurement> getMeasurement() {
//         return poseEstimator.update().flatMap((result) -> {
//             if (result.targetsUsed.get(0).getBestCameraToTarget().getTranslation().getNorm() > Constants.VisionConstants.DISTANCE_FILTER || result.targetsUsed.get(0).getPoseAmbiguity() > Constants.VisionConstants.AMBIGUITY_FILTER) {
//                 return Optional.empty();
//             }

//             // Reject pose estimates outside the field
//             if (result.estimatedPose.toPose2d().getX() < 0 || result.estimatedPose.toPose2d().getX() > Constants.FieldConstants.FIELD_LENGTH ||
//                     result.estimatedPose.toPose2d().getY() < 0 || result.estimatedPose.toPose2d().getY() > Constants.FieldConstants.FIELD_WIDTH) {
//                 return Optional.empty();
//             }

//             return Optional.of(new Measurement(
//                     result.timestampSeconds,
//                     result.estimatedPose,
//                     Constants.VisionConstants.PHOTON_VISION_STD_DEV.forMeasurement(result.targetsUsed.get(0).getBestCameraToTarget().getX(), result.targetsUsed.size())
//             ));
//         });
//     }

//     public interface StandardDeviation {
//         Vector<N3> forMeasurement(double distance, int count);
//     }

//     public void setPoseAlliance() {
//         // Sets the april tag positions depending on which side the robot starts on.
//         if (DriverStation.getAlliance().get() == Alliance.Blue) {
//             poseEstimator.getFieldTags().setOrigin(OriginPosition.kBlueAllianceWallRightSide);
//         } else {
//             poseEstimator.getFieldTags().setOrigin(OriginPosition.kRedAllianceWallRightSide);
//         }
//     }
// }