package frc.robot.Logic;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
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
        boolean useLeftShooterCam = true;
        boolean useLeftCam = true;
        boolean useRightCam = true;

        AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

        // 1
        PhotonCamera rightShooterCam = new PhotonCamera("rightShooterCam");
        // 4
        PhotonCamera leftShooterCam = new PhotonCamera("leftShooterCam");
        // 3
        PhotonCamera leftCam = new PhotonCamera("leftCam");
        // 2
        PhotonCamera rightCam = new PhotonCamera("rightCam");

        public double timeATLastSeen;

        public double visionMaxATDist = Settings.VisionSettings.maxATDistDisabeled;

        Transform3d rightShooterCamTranslation = new Transform3d(
                        new Translation3d(Units.inchesToMeters(-10.015), Units.inchesToMeters(-10.687),
                                        Units.inchesToMeters(24.591)),
                        new Rotation3d(0, Units.degreesToRadians(-30), Units.degreesToRadians(180)));
        Transform3d leftShooterCamTranslation = new Transform3d(
                        new Translation3d(Units.inchesToMeters(-11.046), Units.inchesToMeters(11.318),
                                        Units.inchesToMeters(21.247)),
                        new Rotation3d(0, Units.degreesToRadians(-30), Units.degreesToRadians(180)));
        Transform3d leftCamTranlation = new Transform3d(
                        new Translation3d(Units.inchesToMeters(-10.444), Units.inchesToMeters(11.318),
                                        Units.inchesToMeters(21.247)),
                        new Rotation3d(0, Units.degreesToRadians(-40), Units.degreesToRadians(90)));
        Transform3d rightCamTranslation = new Transform3d(
                        new Translation3d(Units.inchesToMeters(-11.046), Units.inchesToMeters(-11.447),
                                        Units.inchesToMeters(15.878)),
                        new Rotation3d(0, Units.degreesToRadians(-40), Units.degreesToRadians(-90)));

        PhotonPoseEstimator rightShooterPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
                        rightShooterCamTranslation);
        PhotonPoseEstimator leftShooterPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
                        leftShooterCamTranslation);
        PhotonPoseEstimator lefPhotonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
                        leftCamTranlation);
        PhotonPoseEstimator rightPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
                        rightCamTranslation);

        Field2d photonField2d_rightShooter = new Field2d();
        Field2d photonField2d_leftShooter = new Field2d();
        Field2d photoField2d_left = new Field2d();
        Field2d photonField2d_right = new Field2d();

        public Vision() {
                SmartDashboard.putData("photonPose rightShooter", photonField2d_rightShooter);
                SmartDashboard.putData("photonPose leftShooter", photonField2d_leftShooter);
                SmartDashboard.putData("photonPose left", photoField2d_left);
                SmartDashboard.putData("photonPose right", photonField2d_right);

        }

        public void updatePhotonVision() {
                if (Robot.isReal()) {
                        integrateCamera(useRightShooterCam, rightShooterCam, rightShooterPoseEstimator,
                                        photonField2d_rightShooter,
                                        visionMaxATDist, false);
                        integrateCamera(useLeftShooterCam, leftShooterCam, leftShooterPoseEstimator,
                                        photonField2d_leftShooter,
                                        visionMaxATDist, false);
                        integrateCamera(useLeftCam, leftCam, lefPhotonPoseEstimator,
                                        photoField2d_left,
                                        visionMaxATDist, false);
                        integrateCamera(useRightCam, rightCam, rightPoseEstimator, photonField2d_right,
                                        visionMaxATDist, false);
                } else {
                        Robot.drivebase.yagslDrive.addVisionMeasurement(
                                        Robot.drivebase.yagslDrive.getSimulationDriveTrainPose().get(),
                                        Timer.getTimestamp());
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
                                if (useCamera && tag0Dist < maxDistance && poseAmbiguitiy < 0.25) {
                                        if (updateHeadingWithVision) {
                                                Robot.drivebase.yagslDrive.addVisionMeasurement(
                                                                photonPose.get().estimatedPose.toPose2d(),
                                                                photonPose.get().timestampSeconds);
                                                                timeATLastSeen = Timer.getFPGATimestamp();
                                        } else {
                                                Pose2d pvWithoutHeading = new Pose2d(photonPose.get().estimatedPose.getTranslation().toTranslation2d(), Robot.drivebase.yagslDrive.getOdometryHeading());
                                        Robot.drivebase.yagslDrive.addVisionMeasurement(
                                                                pvWithoutHeading,
                                                                photonPose.get().timestampSeconds);
                                        }
                                }

                        }
                }
        }
}
