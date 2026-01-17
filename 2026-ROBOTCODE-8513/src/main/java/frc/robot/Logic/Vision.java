package frc.robot.Logic;


import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Settings;
import edu.wpi.first.apriltag.AprilTagFields;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class Vision {

    boolean useEoghanCam = true;

    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    PhotonCamera eoghanCam = new PhotonCamera("EoghanCam");

    public double visionMaxATDist = Settings.VisionSettings.maxATDistDisabeled;


    Transform3d eoghanCamTransform = new Transform3d(
            new Translation3d(Units.inchesToMeters(6), Units.inchesToMeters(7.3), Units.inchesToMeters(33.5)),
            new Rotation3d(0, Units.degreesToRadians(-26.5), Units.degreesToRadians(0)));

    PhotonPoseEstimator eoghanPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, eoghanCamTransform);

    Field2d photonField2d_processor = new Field2d();


}