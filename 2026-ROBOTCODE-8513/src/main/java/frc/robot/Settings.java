package frc.robot;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class Settings {
    public static AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout
            .loadField(AprilTagFields.k2026RebuiltWelded);

    public static class DrivebaseSettings {

        public static final double maxVelocityMPS = 5;
        public static boolean getPIDValuesFromDashboard = true;

        public static class RotationPIDConstants {
            public static final double kP = 0.1;
            public static final double kI = 0;
            public static final double kD = 0;
        }
    }

    public static class PhysicalRobotValues {
        public static double robotHeight = 30;
    }

    public static class TeleopSettings {
        public static int driverJoystickPort = 1;
        public static int copilotJoystickPort = 2;
        public static int manualJoystickPort = 3;
        public static double joystickDeadband = 0.1;
        public static double specialRotationJoystickDeadband = 0.5;

        // joystick axis
        public static int forwardBackwardsAxis = 1;
        public static int leftRightAxis = 0;
        public static int rotAxis = 4;
        public static int RforwardBackwardsAxis = 4;
        public static int RleftRightAxis = 5;
        public static int rightJoystickX = 5;
        public static int rightJoystickY = 6;

        public static boolean headingJoystickControls = true;

        public static class ButtonIDs {
            // driver controller
            public static int intake = 5;
            public static int faceHub = 7;
            public static int stopIntake = 4;

            // copilot controller
            public static int increaseAngle = 4;
            public static int decreaseAngle = 1;
            public static int moveScorePoseRight = 2;
            public static int moveScorePoseLeft = 3; 
            public static int heightenIntake = 5; 
            public static int lowerIntake = 6; 
            public static int emergencyIntake = 7;
            public static int manualShoot = 8;

        }

    }

    public class IntakeSettings {
        public static double stowPosition = -40;
        public static double deployPosition = 0;
        public static double intakeFudgeFactor = 1;
    }

    public class ShooterSettings {
        public static double hoodPosition = 25.0;
        public static double shooterFudgeFactor = 0.5;
        public static double angleFudgeFactor = 1;
        public static double maxShooterVelocity = 47;
    }

    public class VisionSettings {
        public static double maxATDistDisabeled = 10;
    }

    public class FieldInfo {
        // hub locations

        public static final double width = Units.inchesToMeters(47.0);
        public static final double height = Units.inchesToMeters(72.0); // includes the catcher at the top

        public static final Pose2d hubRedLocation = new Pose2d(
                11.919, 4.029, new Rotation2d(180));
        public static final Pose2d hubBlueLocation = new Pose2d(
                4.621, 4.029, new Rotation2d(0));

        public static final Translation3d blueHubCenterPointTrans3d = new Translation3d(
                aprilTagFieldLayout.getTagPose(26).get().getX() + width / 2.0,
                aprilTagFieldLayout.getFieldWidth() / 2.0,
                height);
        public static final Translation3d redHubCenterPointTrans3d = new Translation3d(
                aprilTagFieldLayout.getTagPose(4).get().getX() + width / 2.0,
                aprilTagFieldLayout.getFieldWidth() / 2.0,
                height);

        public static final Pose2d blueHubCenterPoint = new Pose2d(
                blueHubCenterPointTrans3d.getX(),
                blueHubCenterPointTrans3d.getY(),
                new Rotation2d(0));
        public static final Pose2d redHubCenterPoint = new Pose2d(
                redHubCenterPointTrans3d.getX(),
                redHubCenterPointTrans3d.getY(),
                new Rotation2d(0));

        public static double hubHeight = 10;
    }

    public class AutoSettings {
        public class Thresholds {
            public static double drivebaseRotationTHold = 5;
            public static double hoodPositionTHold = 2;
            public static double shooterVelocityTHold = 100;
        }
    }

    public static void resetTalon(TalonFX m_talonFX) {
        var stickyFaults = m_talonFX.getStickyFault_OverSupplyV().getValue();

        if (stickyFaults) {

            m_talonFX.clearStickyFaults();

            applyConfigs(m_talonFX);
        }
    }

    private static void applyConfigs(TalonFX m_talonFX) {
        var configs = new CurrentLimitsConfigs();
        configs.StatorCurrentLimitEnable = true;
        configs.StatorCurrentLimit = 120;
        configs.SupplyCurrentLimitEnable = true;
        configs.SupplyCurrentLimit = 90;

        m_talonFX.getConfigurator().apply(configs);
    }

}
