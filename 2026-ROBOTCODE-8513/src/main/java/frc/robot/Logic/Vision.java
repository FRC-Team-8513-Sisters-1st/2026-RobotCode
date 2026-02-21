package frc.robot.Logic;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Settings;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

public class Vision {

        boolean updateHeadingWithVision = true;

        boolean useRightShooterCam = true;

        AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

        PhotonCamera rightShooterCam = new PhotonCamera("rightShooterCam");

        public double timeATLastSeen;

        public double visionMaxATDist = Settings.VisionSettings.maxATDistDisabeled;

        Transform3d rightShooterCamTranslation = new Transform3d(
                        new Translation3d(Units.inchesToMeters(-12), Units.inchesToMeters(-12),
                                        Units.inchesToMeters(13)),
                        new Rotation3d(0, -25, Units.degreesToRadians(180)));

        PhotonPoseEstimator rightShooterPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, rightShooterCamTranslation);

        Field2d photonField2d_processor = new Field2d();

        public void updatePhotonVision() {
                if (Robot.isReal()) {
                        integrateCamera(useRightShooterCam, rightShooterCam, rightShooterPoseEstimator, photonField2d_processor,
                                        visionMaxATDist, false);
                } else {
                        Robot.drivebase.yagslDrive.addVisionMeasurement(Robot.drivebase.yagslDrive.getSimulationDriveTrainPose().get(), Timer.getTimestamp());
                }

        }

        public void integrateCamera(boolean useCamera, PhotonCamera camera, PhotonPoseEstimator estimator,
                        Field2d photonField, double maxDistance, boolean updateLastTimeSeen) {
                for (var result : camera.getAllUnreadResults()) {
                        Optional<EstimatedRobotPose> photonPose = estimator.estimateLowestAmbiguityPose(result);

                       
                        if (photonPose.isPresent()) {
                                photonField.setRobotPose(photonPose.get().estimatedPose.toPose2d());

                                double tag0Dist = result.getBestTarget().bestCameraToTarget
                                                .getTranslation()
                                                .getNorm();
                                SmartDashboard.putNumber("Tag 0", tag0Dist);

                                double poseAmbiguitiy = result.getBestTarget().getPoseAmbiguity();
                                if (useCamera && tag0Dist < maxDistance && poseAmbiguitiy < 0.15) {
                                        if (updateHeadingWithVision) {
                                                Robot.drivebase.yagslDrive.addVisionMeasurement(
                                                                photonPose.get().estimatedPose.toPose2d(),
                                                                photonPose.get().timestampSeconds);
                                        }
                                }

                        }
                }
        }
}
