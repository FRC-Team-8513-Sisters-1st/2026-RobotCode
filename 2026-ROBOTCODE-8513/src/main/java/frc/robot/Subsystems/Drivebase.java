package frc.robot.Subsystems;

import java.io.File;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Robot;
import frc.robot.Settings;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

public class Drivebase {
    public SwerveDrive yagslDrive;
    public Rotation2d goalHeading = new Rotation2d();
    public PIDController rotationPidController = new PIDController(
            frc.robot.Settings.DrivebaseSettings.RotationPIDConstants.kP,
            frc.robot.Settings.DrivebaseSettings.RotationPIDConstants.kI,
            frc.robot.Settings.DrivebaseSettings.RotationPIDConstants.kD);

    // path following variables
    PathPlannerTrajectory traj;
    public boolean loadedPathHasStarted = false;
    PathPlannerPath path;
    public String pathName = "";
    double elapsedTime;
    double timePathStarted;
    public PathPlannerTrajectoryState trajGoalState = new PathPlannerTrajectoryState();
    Field2d trajGoalPosition = new Field2d();
    double otfEndVelocity = 0;
    public boolean forcePathHeading = false;
    double dvr;
    public Pose2d goalAimPose;

    // edit these values and put in settings later
    public PIDController followPathXController = new PIDController(10, 0, 0);

    public PIDController followPathYController = new PIDController(10, 0, 0);

    public double timeOfFlight;

    public double aimFudgeFactor = 0;

    public Drivebase() {
        File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
        try {
            yagslDrive = new SwerveParser(swerveJsonDirectory)
                    .createSwerveDrive(Settings.DrivebaseSettings.maxVelocityMPS,
                            new Pose2d(8.25, 4, new Rotation2d(0)));
        } catch (Exception e) {
            e.printStackTrace();
        }

        // sets the goal aim pose to the hub (when the aim pose is change bc of
        // velocity, it is updated)
        if (Robot.onRed) {
            goalAimPose = Settings.FieldInfo.redHubCenterPoint;

        } else {
            goalAimPose = Settings.FieldInfo.blueHubCenterPoint;

        }
    }

    public void driveFacingHeading(Translation2d translation2d, Rotation2d heading, boolean fR) {
        double angleError = Robot.drivebase.yagslDrive.getOdometryHeading().minus(heading).getDegrees();
        double rotationCorrection = rotationPidController.calculate(angleError, 0);

        Robot.drivebase.yagslDrive.drive(translation2d,
                rotationCorrection,
                fR,
                false);

    }

    public void initPath(String pathNameIn) {
        pathName = pathNameIn;

        try {
            path = PathPlannerPath.fromPathFile(pathName);
        } catch (Exception e) {
            System.out.println("Error in loading path");
            e.printStackTrace();
        }

        // flip the path if on red
        if (Robot.onRed) {
            path = path.flipPath();
        }

        try {
            traj = path.getIdealTrajectory(RobotConfig.fromGUISettings()).get();
        } catch (Exception e) {
            System.out.println("Error in trajectory generation");
            e.printStackTrace();
        }

        if (Robot.isSimulation()) {
            yagslDrive.resetOdometry(path.getStartingHolonomicPose().get());
        }

        loadedPathHasStarted = false;

    }

    public boolean followLoadedPath() {
        double correctionInXV;
        double correctionInYV;

        if (!loadedPathHasStarted) {
            timePathStarted = Timer.getFPGATimestamp();
            loadedPathHasStarted = true;
        }

        elapsedTime = Timer.getFPGATimestamp() - timePathStarted;

        if (elapsedTime > traj.getTotalTimeSeconds()) {
            return true;
        } else {
            PathPlannerTrajectoryState pathGoalState = traj.sample(elapsedTime);

            double trajGoalX = pathGoalState.pose.getX();
            double trajGoalY = pathGoalState.pose.getY();
            Rotation2d trajGoalHeading = pathGoalState.pose.getRotation();
            Robot.dashboard.trajField2d.setRobotPose(trajGoalX, trajGoalY, trajGoalHeading);

            correctionInXV = followPathXController.calculate(Robot.drivebase.yagslDrive.getPose().getX(),
                    trajGoalX);
            correctionInYV = followPathYController.calculate(Robot.drivebase.yagslDrive.getPose().getY(),
                    trajGoalY);

            double xV = pathGoalState.fieldSpeeds.vxMetersPerSecond + correctionInXV;
            double yV = pathGoalState.fieldSpeeds.vyMetersPerSecond + correctionInYV;

            Translation2d correctedPathFollowingTranslation = new Translation2d(xV, yV);

            driveFacingHeading(correctedPathFollowingTranslation, trajGoalHeading, true);

            return false;

        }

    }

    public double getPowerToFaceHub() {
        Rotation2d angleToHub;

        if (Robot.onRed) {
            // red hub location
            goalAimPose = offsetPose2dByVelocity(Settings.FieldInfo.redHubCenterPoint);
            angleToHub = yagslDrive.getPose().minus(goalAimPose)
                    .getTranslation().getAngle();
        } else {
            // blue hub location
            goalAimPose = offsetPose2dByVelocity(Settings.FieldInfo.blueHubCenterPoint);
            angleToHub = yagslDrive.getPose().minus(goalAimPose)
                    .getTranslation().getAngle()
                    .plus(new Rotation2d(Math.PI));
        }
        goalHeading = angleToHub.plus(new Rotation2d(aimFudgeFactor));
        dvr = rotationPidController.calculate(yagslDrive.getOdometryHeading().minus(angleToHub).getDegrees(), 0);

        return dvr;
    }

    // calculates the tof
    public double timeOfFlight() {
        timeOfFlight = 1;
        return timeOfFlight;
    }

    // returns the new Pose2d score location
    double shotDisplacement;

    public Pose2d offsetPose2dByVelocity(Pose2d originalPose2d) {
        double rVX = yagslDrive.getFieldVelocity().vxMetersPerSecond;
        double rVY = yagslDrive.getFieldVelocity().vyMetersPerSecond;
        double x = rVX * timeOfFlight;
        double y = rVY * timeOfFlight;
        Transform2d transform = new Transform2d(x, y, new Rotation2d());
        return originalPose2d.transformBy(transform);
    }

    public double getDistanceBetweenTwoPoses(Pose2d pose1, Pose2d pose2) {
        double x = pose2.getX() - pose1.getX();
        double y = pose2.getY() - pose1.getY();
        double distance = Math.sqrt(x * x + y * y);
        return distance;
    }

    public void faceHub() {
        double powerToAngleToHub = getPowerToFaceHub();
        yagslDrive.drive(new Translation2d(), powerToAngleToHub, true, false);
    }

    public double drivingOverBump(boolean onRed) {
        if (onRed) {
            if (Robot.drivebase.yagslDrive.getPose().getX() > 11
                    && Robot.drivebase.yagslDrive.getPose().getX() < 12.7) {
                return 0.3;
            } else {
                return 0.8;
            }
        } else {
            if (Robot.drivebase.yagslDrive.getPose().getX() > 3.85
                    && Robot.drivebase.yagslDrive.getPose().getX() < 5.4) {
                return 0.3;
            } else {
                return 0.8;
            }
        }
    }
}
