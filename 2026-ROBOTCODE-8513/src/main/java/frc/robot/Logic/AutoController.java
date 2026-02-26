package frc.robot.Logic;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Settings;
import frc.robot.Logic.Enums.AutoRoutines;
import frc.robot.Logic.Enums.HopperStates;
import frc.robot.Logic.Enums.IntakeStates;
import frc.robot.Logic.Enums.KickerStates;
import frc.robot.Logic.Enums.ShooterStates;

public class AutoController {

    public SendableChooser<String> autoSelector;
    public AutoRoutines autoRoutine = AutoRoutines.DoNothing;
    public AutoRoutines autoToReturnTo = AutoRoutines.DoNothing;

    public int autoStep;
    double timeStepStarted = 0;
    int bumpTholdCounter = 0;

    public AutoController() {
        // create auto selector with each enum option
        autoSelector = new SendableChooser<>();
        autoSelector.setDefaultOption(AutoRoutines.values()[0].toString(), AutoRoutines.values()[0].toString());
        for (int i = 1; i < AutoRoutines.values().length; i++) {
            if (AutoRoutines.values()[i].toString().charAt(0) != '~') {
                autoSelector.addOption(AutoRoutines.values()[i].toString(), AutoRoutines.values()[i].toString());
            }

        }
        SmartDashboard.putData("Auton Selector", autoSelector);
    }

    public void initAuto() {
        Elastic.selectTab("Autonomous");
        updateAutoRoutineFromDashboard();
        autoStep = 0;
        timeStepStarted = Timer.getFPGATimestamp();
    }

    public void autoPeriodic() {

        switch (autoRoutine) {
            case DoNothing:
                Robot.drivebase.yagslDrive.lockPose();
                break;

            case Depot_OneCycle:
                switch (autoStep) {
                    case 0:
                        Robot.shooter.shooterState = ShooterStates.stationary;
                        Robot.hopper.hopperState = HopperStates.stationary;
                        Robot.kicker.kickerState = KickerStates.stationary;
                        Robot.intake.intakeState = IntakeStates.stationaryDeployed;

                        Robot.drivebase.initPath("Depot-OneCycle");

                        // path is over
                        timeStepStarted = Timer.getFPGATimestamp();
                        autoStep = 10;
                        break;

                    case 10:
                        if (Robot.drivebase.followLoadedPath()) {
                            timeStepStarted = Timer.getFPGATimestamp();
                            autoStep = 15;
                        }

                        if (Timer.getFPGATimestamp() - timeStepStarted >= 2.8) {
                            Robot.intake.intakeState = IntakeStates.intaking;
                        }

                        if (Timer.getFPGATimestamp() - timeStepStarted >= 6.14) {
                            Robot.intake.intakeState = IntakeStates.stationaryDeployed;
                            Robot.shooter.shooterState = ShooterStates.shooting;
                        }

                        break;
                    case 15:
                        Robot.drivebase.faceHub();
                        if (Robot.shooter.readyToShootInHub()) {
                            autoStep = 20;
                        }
                        break;
                    case 20:
                        Robot.drivebase.faceHub();
                        Robot.hopper.hopperState = HopperStates.indexing;
                        Robot.kicker.kickerState = KickerStates.shooting;
                        break;
                }

            case Outpost_OneCycle:
                switch (autoStep) {
                    case 0:
                        autoToReturnTo = AutoRoutines.Outpost_OneCycle;
                        autoRoutine = AutoRoutines.GoOverBump;

                        Robot.shooter.shooterState = ShooterStates.stationary;
                        Robot.hopper.hopperState = HopperStates.stationary;
                        Robot.kicker.kickerState = KickerStates.stationary;
                        Robot.intake.intakeState = IntakeStates.stationaryDeployed;

                        Robot.drivebase.initPath("Outpost-OneCycle");

                        // path is over
                        timeStepStarted = Timer.getFPGATimestamp();
                        autoStep = 10;
                        break;

                    case 10:
                        if (Robot.drivebase.followLoadedPath()) {
                            timeStepStarted = Timer.getFPGATimestamp();
                            autoStep = 15;
                        }

                        if (Timer.getFPGATimestamp() - timeStepStarted >= 2.8) {
                            Robot.intake.intakeState = IntakeStates.intaking;
                        }

                        if (Timer.getFPGATimestamp() - timeStepStarted >= 6.14) {
                            Robot.intake.intakeState = IntakeStates.stationaryDeployed;
                            Robot.shooter.shooterState = ShooterStates.shooting;
                        }

                        break;
                    case 15:
                        Robot.drivebase.faceHub();
                        if (Robot.shooter.readyToShootInHub()) {
                            autoStep = 20;
                        }
                        break;
                    case 20:
                        Robot.drivebase.faceHub();
                        Robot.hopper.hopperState = HopperStates.indexing;
                        Robot.kicker.kickerState = KickerStates.shooting;
                        break;
                }

            case TestAuto:

                switch (autoStep) {
                    case 0:
                        Robot.shooter.shooterState = ShooterStates.stationary;
                        Robot.hopper.hopperState = HopperStates.stationary;
                        Robot.kicker.kickerState = KickerStates.stationary;
                        Robot.intake.intakeState = IntakeStates.stowed;

                        Robot.drivebase.initPath("TestAuto-step5");

                        // path is over
                        timeStepStarted = Timer.getFPGATimestamp();
                        autoStep = 10;
                        break;

                    case 10:
                        Robot.shooter.shooterState = ShooterStates.shooting;
                        Robot.hopper.hopperState = HopperStates.indexing;
                        Robot.kicker.kickerState = KickerStates.shooting;
                        Robot.intake.intakeState = IntakeStates.stowed;

                        if (Robot.drivebase.followLoadedPath()) {
                            timeStepStarted = Timer.getFPGATimestamp();
                            autoStep = 12;
                        }
                        break;
                    case 12:
                        Robot.drivebase.yagslDrive.lockPose();
                        if (Timer.getFPGATimestamp() - timeStepStarted > 3) {
                            autoStep = 15;
                        }
                        break;

                    case 15:
                        Robot.shooter.shooterState = ShooterStates.stationary;
                        Robot.hopper.hopperState = HopperStates.stationary;
                        Robot.kicker.kickerState = KickerStates.stationary;
                        Robot.intake.intakeState = IntakeStates.stowed;

                        Robot.drivebase.initPath("TestAuto-step15");

                        // path is over
                        timeStepStarted = Timer.getFPGATimestamp();
                        autoStep = 20;
                        break;
                    case 20:
                        if (Robot.drivebase.followLoadedPath()) {
                            autoStep = 25;
                        }
                        break;
                    case 25:
                        Robot.drivebase.yagslDrive.lockPose();
                        break;
                }
                break;
            case GoOverBump:
                switch (autoStep) {
                    case 0:
                        Robot.shooter.shooterState = ShooterStates.stationary;
                        Robot.hopper.hopperState = HopperStates.stationary;
                        Robot.kicker.kickerState = KickerStates.stationary;
                        Robot.intake.intakeState = IntakeStates.stationaryDeployed;

                        // path is over
                        timeStepStarted = Timer.getFPGATimestamp();
                        autoStep = 10;
                        break;
                    case 10:
                        if (Robot.onRed) {
                            Robot.drivebase.driveFacingHeading(new Translation2d(-1, 0), new Rotation2d(Math.PI), true);
                        } else {
                            Robot.drivebase.driveFacingHeading(new Translation2d(1, 0), new Rotation2d(0), true);
                        }
                        if (Math.abs(Robot.drivebase.yagslDrive.getPitch()
                                .getDegrees()) > Settings.AutoSettings.Thresholds.autoDetectedBumpPitchTHold) {
                            bumpTholdCounter++;
                        } else {
                            bumpTholdCounter = 0;
                        }
                        if (bumpTholdCounter >= Settings.AutoSettings.Thresholds.autoDetectedBumpPitchCount) {
                            bumpTholdCounter = 0;
                            autoStep = 15;
                        }
                        break;
                    case 15:
                        if (Math.abs(Robot.drivebase.yagslDrive.getPitch()
                                .getDegrees()) < Settings.AutoSettings.Thresholds.autoDetectedBumpPitchTHold) {
                            bumpTholdCounter++;
                        } else {
                            bumpTholdCounter = 0;
                        }
                        if (Robot.onRed) {
                            Robot.drivebase.driveFacingHeading(new Translation2d(-1, 0), new Rotation2d(Math.PI),
                                    true);
                        } else {
                            Robot.drivebase.driveFacingHeading(new Translation2d(1, 0), new Rotation2d(0), true);
                        }

                        if (bumpTholdCounter > Settings.AutoSettings.Thresholds.autoDetectedBumpPitchCount) {
                            bumpTholdCounter = 0;
                            autoStep = 5;
                            autoRoutine = autoToReturnTo;
                        }
                        break;
                }
            case Depot:
                switch (autoStep) {
                    case 0:
                        ifNoCameraAssumeRobotPos(new Pose2d(3.560, 5.504, new Rotation2d(180)));

                        Robot.shooter.shooterState = ShooterStates.stationary;
                        Robot.hopper.hopperState = HopperStates.stationary;
                        Robot.kicker.kickerState = KickerStates.stationary;
                        Robot.intake.intakeState = IntakeStates.intaking;

                        Robot.drivebase.initPath("Depot_Depot_Pt1");

                        // path is over
                        timeStepStarted = Timer.getFPGATimestamp();
                        autoStep = 10;
                        break;

                    case 10:
                        Robot.shooter.shooterState = ShooterStates.stationary;
                        Robot.hopper.hopperState = HopperStates.stationary;
                        Robot.kicker.kickerState = KickerStates.stationary;
                        Robot.intake.intakeState = IntakeStates.intaking;

                        if (Robot.drivebase.followLoadedPath()) {
                            timeStepStarted = Timer.getFPGATimestamp();
                            autoStep = 12;
                        }
                        break;
                    case 12:
                        Robot.drivebase.yagslDrive.lockPose();
                        if (Timer.getFPGATimestamp() - timeStepStarted > 2) {
                            autoStep = 15;
                        }
                        break;

                    case 15:
                        Robot.shooter.shooterState = ShooterStates.shooting;
                        Robot.hopper.hopperState = HopperStates.stationary;
                        Robot.kicker.kickerState = KickerStates.stationary;
                        Robot.intake.intakeState = IntakeStates.intaking;

                        Robot.drivebase.initPath("Depot_Depot_Pt2");

                        // path is over
                        timeStepStarted = Timer.getFPGATimestamp();
                        autoStep = 20;
                        break;
                    case 20:

                        if (Robot.drivebase.followLoadedPath()) {
                            autoStep = 25;
                        }
                        break;
                    case 25:
                        Robot.shooter.shooterState = ShooterStates.shooting;
                        Robot.hopper.hopperState = HopperStates.indexing;
                        Robot.kicker.kickerState = KickerStates.shooting;
                        Robot.intake.intakeState = IntakeStates.shooting;

                        Robot.drivebase.yagslDrive.lockPose();
                        break;
                }
                break;
            case Outpost:
                switch (autoStep) {
                    case 0:
                        ifNoCameraAssumeRobotPos(new Pose2d(3.948, 2.929, new Rotation2d(180)));

                        Robot.shooter.shooterState = ShooterStates.shooting;
                        Robot.hopper.hopperState = HopperStates.stationary;
                        Robot.kicker.kickerState = KickerStates.stationary;
                        Robot.intake.intakeState = IntakeStates.stationaryDeployed;

                        Robot.drivebase.initPath("Outpost_Outpost_Pt1");

                        // path is over
                        timeStepStarted = Timer.getFPGATimestamp();
                        autoStep = 10;
                        break;

                    case 10:
                        Robot.shooter.shooterState = ShooterStates.shooting;
                        Robot.hopper.hopperState = HopperStates.stationary;
                        Robot.kicker.kickerState = KickerStates.stationary;
                        Robot.intake.intakeState = IntakeStates.stowed;

                        if (Robot.drivebase.followLoadedPath()) {
                            timeStepStarted = Timer.getFPGATimestamp();
                            autoStep = 12;
                        }
                        break;
                    case 12:
                        Robot.drivebase.yagslDrive.lockPose();
                        if (Timer.getFPGATimestamp() - timeStepStarted > 2) {
                            autoStep = 15;
                        }
                        break;

                    case 15:
                        Robot.shooter.shooterState = ShooterStates.shooting;
                        Robot.hopper.hopperState = HopperStates.stationary;
                        Robot.kicker.kickerState = KickerStates.stationary;
                        Robot.intake.intakeState = IntakeStates.stowed;

                        Robot.drivebase.initPath("Outpost_Outpost_Pt2");

                        // path is over
                        timeStepStarted = Timer.getFPGATimestamp();
                        autoStep = 20;
                        break;
                    case 20:

                        if (Robot.drivebase.followLoadedPath()) {
                            autoStep = 25;
                        }
                        break;
                    case 25:
                        Robot.shooter.shooterState = ShooterStates.shooting;
                        Robot.hopper.hopperState = HopperStates.indexing;
                        Robot.kicker.kickerState = KickerStates.shooting;
                        Robot.intake.intakeState = IntakeStates.intaking;

                        Robot.drivebase.yagslDrive.lockPose();
                        break;
                }
                break;
        }

        Robot.intake.setMotorPower();
        Robot.hopper.setMotorPower();
        Robot.kicker.setMotorPower();
        Robot.shooter.setMotorPower();
    }

    public void updateAutoRoutineFromDashboard() {
        try {
            autoRoutine = AutoRoutines.valueOf(autoSelector.getSelected());
        } catch (Exception e) {
            autoRoutine = AutoRoutines.DoNothing;
        }
    }

    // asumes the robot pose if there has been no apriltags
    public void ifNoCameraAssumeRobotPos(Pose2d autoStartPose) {
        if (Robot.vision.timeATLastSeen == Timer.getFPGATimestamp()) {
            Robot.drivebase.yagslDrive.resetOdometry(autoStartPose);
        }
    }

}
