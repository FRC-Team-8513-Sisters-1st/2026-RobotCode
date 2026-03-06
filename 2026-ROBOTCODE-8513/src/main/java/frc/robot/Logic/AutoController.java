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
        Robot.intake.intakeDeployController.reset(Robot.intake.intakeDeployMotor.getPosition().getValueAsDouble());
        Robot.drivebase.rotationPidController.reset(0);

    }

    public void autoPeriodic() {

        switch (autoRoutine) {
            case DoNothing:
                Robot.drivebase.yagslDrive.lockPose();
                break;

            case MoveForward1Meter:
                switch (autoStep) {
                    case 0:
                        Robot.shooter.shooterState = ShooterStates.stationary;
                        Robot.hopper.hopperState = HopperStates.stationary;
                        Robot.kicker.kickerState = KickerStates.stationary;
                        Robot.intake.intakeState = IntakeStates.stationaryDeployed;

                        Robot.drivebase.initPath("Move Forward");

                        // path is over
                        timeStepStarted = Timer.getFPGATimestamp();
                        autoStep = 10;
                        break;

                    case 10:
                        if (Robot.drivebase.followLoadedPath()) {
                            timeStepStarted = Timer.getFPGATimestamp();
                            Robot.drivebase.yagslDrive.lockPose();
                        }
                        break;
                }
                break;

            case MoveForward1MeterComeBack:
                switch (autoStep) {
                    case 0:
                        Robot.shooter.shooterState = ShooterStates.stationary;
                        Robot.hopper.hopperState = HopperStates.stationary;
                        Robot.kicker.kickerState = KickerStates.stationary;
                        Robot.intake.intakeState = IntakeStates.stationaryDeployed;

                        Robot.drivebase.initPath("Move Forward");

                        // path is over
                        timeStepStarted = Timer.getFPGATimestamp();
                        autoStep = 10;
                        break;

                    case 10:
                        if (Robot.drivebase.followLoadedPath()) {
                            timeStepStarted = Timer.getFPGATimestamp();
                            autoStep = 15;
                        }
                        break;

                    case 15:
                        if (Timer.getFPGATimestamp() - timeStepStarted >= 2.8) {
                            Robot.drivebase.initPath("Come Back");
                            autoStep = 20;
                            Robot.drivebase.yagslDrive.lockPose();
                        }
                        break;
                }
                break;

            case RotateWhileDriving:
                switch (autoStep) {
                    case 0:
                        Robot.shooter.shooterState = ShooterStates.stationary;
                        Robot.hopper.hopperState = HopperStates.stationary;
                        Robot.kicker.kickerState = KickerStates.stationary;
                        Robot.intake.intakeState = IntakeStates.stationaryDeployed;

                        Robot.drivebase.initPath("Drive While Rotating");

                        // path is over
                        timeStepStarted = Timer.getFPGATimestamp();
                        autoStep = 10;
                        break;

                    case 10:
                        if (Robot.drivebase.followLoadedPath()) {
                            timeStepStarted = Timer.getFPGATimestamp();
                            Robot.drivebase.yagslDrive.lockPose();

                        }
                        break;
                }
                break;

            case DriveAtAnAngle:
                switch (autoStep) {
                    case 0:
                        Robot.shooter.shooterState = ShooterStates.stationary;
                        Robot.hopper.hopperState = HopperStates.stationary;
                        Robot.kicker.kickerState = KickerStates.stationary;
                        Robot.intake.intakeState = IntakeStates.stationaryDeployed;

                        Robot.drivebase.initPath("Drive At Angle");

                        // path is over
                        timeStepStarted = Timer.getFPGATimestamp();
                        autoStep = 10;
                        break;

                    case 10:
                        if (Robot.drivebase.followLoadedPath()) {
                            timeStepStarted = Timer.getFPGATimestamp();
                            Robot.drivebase.yagslDrive.lockPose();
                        }
                        break;
                }
                break;

            case Depot_OneCycle:
                switch (autoStep) {
                    case 0:
                        autoToReturnTo = AutoRoutines.Depot_OneCycle;
                        autoRoutine = AutoRoutines.OliviaAttemptGoOverBump;

                        Robot.shooter.shooterState = ShooterStates.stationary;
                        Robot.hopper.hopperState = HopperStates.stationary;
                        Robot.kicker.kickerState = KickerStates.stationary;
                        Robot.intake.intakeState = IntakeStates.stationaryDeployed;

                        Robot.drivebase.initPath("Depot-OneCycle");

                        // path is over
                        timeStepStarted = Timer.getFPGATimestamp();
                        autoStep = 0;
                        break;

                    case 5:
                        if (Robot.drivebase.followLoadedPath()) {
                            timeStepStarted = Timer.getFPGATimestamp();
                            autoStep = 15;
                        }

                        if (Timer.getFPGATimestamp() - timeStepStarted >= 0.5) {
                            Robot.intake.intakeState = IntakeStates.intaking;
                        }

                        if (Timer.getFPGATimestamp() - timeStepStarted >= 7) {
                            Robot.intake.intakeState = IntakeStates.stationaryDeployed;
                            Robot.shooter.shooterState = ShooterStates.shooting;
                        }

                        break;
                    case 15:
                        Robot.drivebase.faceHub();
                        if (Robot.shooter.readyToShootInHub()) {
                            autoStep = 20;
                            Robot.teleop.timeIntakeShootingButtonPressed = Timer.getFPGATimestamp();

                        }
                        break;
                    case 20:
                        Robot.drivebase.faceHub();
                        Robot.hopper.hopperState = HopperStates.indexing;
                        Robot.kicker.kickerState = KickerStates.shooting;
                        Robot.intake.intakeState = IntakeStates.shooting;
                        break;
                }
                break;
            case Outpost_OneCycle:
                switch (autoStep) {
                    case 0:
                        autoToReturnTo = AutoRoutines.Outpost_OneCycle;
                        autoRoutine = AutoRoutines.OliviaAttemptGoOverBump;

                        Robot.shooter.shooterState = ShooterStates.stationary;
                        Robot.hopper.hopperState = HopperStates.stationary;
                        Robot.kicker.kickerState = KickerStates.stationary;
                        Robot.intake.intakeState = IntakeStates.stationaryDeployed;

                        Robot.drivebase.initPath("Outpost-OneCycle");

                        // path is over
                        timeStepStarted = Timer.getFPGATimestamp();
                        autoStep = 0;
                        break;

                    case 5:
                        if (Robot.drivebase.followLoadedPath()) {
                            timeStepStarted = Timer.getFPGATimestamp();
                            autoStep = 15;
                        }

                        if (Timer.getFPGATimestamp() - timeStepStarted >= 0.5) {
                            Robot.intake.intakeState = IntakeStates.intaking;
                        }

                        if (Timer.getFPGATimestamp() - timeStepStarted >= 7.5) {
                            Robot.intake.intakeState = IntakeStates.stationaryDeployed;
                            Robot.shooter.shooterState = ShooterStates.shooting;
                        }

                        break;
                    case 15:
                        Robot.drivebase.faceHub();
                        if (Robot.shooter.readyToShootInHub()) {
                            autoStep = 20;
                            Robot.teleop.timeIntakeShootingButtonPressed = Timer.getFPGATimestamp();
                            timeStepStarted = Timer.getFPGATimestamp();

                        }
                        break;
                    case 20:
                        Robot.drivebase.faceHub();
                        Robot.hopper.hopperState = HopperStates.indexing;
                        Robot.kicker.kickerState = KickerStates.shooting;
                        Robot.intake.intakeState = IntakeStates.shooting;
                        if (Timer.getFPGATimestamp() - timeStepStarted < 0.4) {
                            Robot.intake.intakeState = IntakeStates.shooting;
                        }
                        break;
                }
                break;
            case Outpost_TwoCycle:
                switch (autoStep) {
                    case 0:
                        autoToReturnTo = AutoRoutines.Outpost_TwoCycle;
                        autoRoutine = AutoRoutines.OliviaAttemptGoOverBump;

                        Robot.shooter.shooterState = ShooterStates.stationary;
                        Robot.hopper.hopperState = HopperStates.stationary;
                        Robot.kicker.kickerState = KickerStates.stationary;
                        Robot.intake.intakeState = IntakeStates.stationaryDeployed;

                        Robot.drivebase.initPath("Outpost-OneCycle");

                        // path is over
                        timeStepStarted = Timer.getFPGATimestamp();
                        autoStep = 0;
                        break;

                    case 5:
                        if (Robot.drivebase.followLoadedPath()) {
                            timeStepStarted = Timer.getFPGATimestamp();
                            autoStep = 15;
                        }

                        if (Timer.getFPGATimestamp() - timeStepStarted >= 0.5) {
                            Robot.intake.intakeState = IntakeStates.intaking;
                        }

                        if (Timer.getFPGATimestamp() - timeStepStarted >= 6.46) {
                            Robot.intake.intakeState = IntakeStates.stationaryDeployed;
                            Robot.shooter.shooterState = ShooterStates.shooting;
                        }

                        break;
                    case 15:
                        Robot.drivebase.faceHub();
                        if (Robot.shooter.readyToShootInHub()) {
                            timeStepStarted = Timer.getFPGATimestamp();
                            autoStep = 20;
                            Robot.teleop.timeIntakeShootingButtonPressed = Timer.getFPGATimestamp();
                        }
                        break;
                    case 20:
                        Robot.drivebase.faceHub();

                        Robot.hopper.hopperState = HopperStates.indexing;
                        Robot.kicker.kickerState = KickerStates.shooting;
                        Robot.intake.intakeState = IntakeStates.shooting;

                        if (Timer.getFPGATimestamp() - timeStepStarted >= 5) {
                            Robot.drivebase.initPath("Outpost-OneCycle-Reset");
                            autoStep = 30;
                        }
                        break;

                    case 30:
                        Robot.shooter.shooterState = ShooterStates.stationary;
                        Robot.hopper.hopperState = HopperStates.stationary;
                        Robot.kicker.kickerState = KickerStates.stationary;
                        Robot.intake.intakeState = IntakeStates.intaking;

                        if (Robot.drivebase.followLoadedPath()) {
                            timeStepStarted = Timer.getFPGATimestamp();
                            autoStep = 35;
                        }
                        break;
                    case 35:
                        autoToReturnTo = AutoRoutines.Outpost_TwoCycle;
                        autoRoutine = AutoRoutines.OliviaAttemptGoOverBump;

                        Robot.shooter.shooterState = ShooterStates.stationary;
                        Robot.hopper.hopperState = HopperStates.stationary;
                        Robot.kicker.kickerState = KickerStates.stationary;
                        Robot.intake.intakeState = IntakeStates.stationaryDeployed;

                        Robot.drivebase.initPath("Outpost-OneCycle");

                        // path is over
                        timeStepStarted = Timer.getFPGATimestamp();
                        autoStep = 0;
                        break;
                }
                break;
                case Depot_TwoCycle:
                switch (autoStep) {
                    case 0:
                        autoToReturnTo = AutoRoutines.Depot_TwoCycle;
                        autoRoutine = AutoRoutines.OliviaAttemptGoOverBump;

                        Robot.shooter.shooterState = ShooterStates.stationary;
                        Robot.hopper.hopperState = HopperStates.stationary;
                        Robot.kicker.kickerState = KickerStates.stationary;
                        Robot.intake.intakeState = IntakeStates.stationaryDeployed;

                        Robot.drivebase.initPath("Depot_TwoCycle");

                        // path is over
                        timeStepStarted = Timer.getFPGATimestamp();
                        autoStep = 0;
                        break;

                    case 5:
                        if (Robot.drivebase.followLoadedPath()) {
                            timeStepStarted = Timer.getFPGATimestamp();
                            autoStep = 15;
                        }

                       if (Timer.getFPGATimestamp() - timeStepStarted >= 0.5) {
                            Robot.intake.intakeState = IntakeStates.intaking;
                        }

                        if (Timer.getFPGATimestamp() - timeStepStarted >= 7) {
                            Robot.intake.intakeState = IntakeStates.stationaryDeployed;
                            Robot.shooter.shooterState = ShooterStates.shooting;
                        }

                        break;
                    case 15:
                        Robot.drivebase.faceHub();
                        if (Robot.shooter.readyToShootInHub()) {
                            timeStepStarted = Timer.getFPGATimestamp();
                            autoStep = 20;
                            Robot.teleop.timeIntakeShootingButtonPressed = Timer.getFPGATimestamp();
                        }
                        break;
                    case 20:
                        Robot.drivebase.faceHub();

                        Robot.hopper.hopperState = HopperStates.indexing;
                        Robot.kicker.kickerState = KickerStates.shooting;
                        Robot.intake.intakeState = IntakeStates.shooting;

                        if (Timer.getFPGATimestamp() - timeStepStarted >= 5) {
                            Robot.drivebase.initPath("Depot-OneCycle-Reset");
                            autoStep = 30;
                        }
                        break;

                    case 30:
                        Robot.shooter.shooterState = ShooterStates.stationary;
                        Robot.hopper.hopperState = HopperStates.stationary;
                        Robot.kicker.kickerState = KickerStates.stationary;
                        Robot.intake.intakeState = IntakeStates.intaking;

                        if (Robot.drivebase.followLoadedPath()) {
                            timeStepStarted = Timer.getFPGATimestamp();
                            autoStep = 35;
                        }
                        break;
                    case 35:
                        autoToReturnTo = AutoRoutines.Depot_TwoCycle;
                        autoRoutine = AutoRoutines.OliviaAttemptGoOverBump;

                        Robot.shooter.shooterState = ShooterStates.stationary;
                        Robot.hopper.hopperState = HopperStates.stationary;
                        Robot.kicker.kickerState = KickerStates.stationary;
                        Robot.intake.intakeState = IntakeStates.stationaryDeployed;

                        Robot.drivebase.initPath("Depot_TwoCycle");

                        // path is over
                        timeStepStarted = Timer.getFPGATimestamp();
                        autoStep = 0;
                        break;
                }
                break;
            case Depot_FullAcross_OneCycle:
                switch (autoStep) {
                    case 0:

                        autoToReturnTo = AutoRoutines.Depot_FullAcross_OneCycle;
                        autoRoutine = AutoRoutines.OliviaAttemptGoOverBump;

                        Robot.shooter.shooterState = ShooterStates.stationary;
                        Robot.hopper.hopperState = HopperStates.stationary;
                        Robot.kicker.kickerState = KickerStates.stationary;
                        Robot.intake.intakeState = IntakeStates.stationaryDeployed;

                        Robot.drivebase.initPath("Depot-fullAcross-OneCycle");

                        // path is over
                        timeStepStarted = Timer.getFPGATimestamp();
                        autoStep = 0;
                        break;

                    case 5:
                        if (Robot.drivebase.followLoadedPath()) {
                            timeStepStarted = Timer.getFPGATimestamp();
                            autoStep = 15;
                        }

                        if (Timer.getFPGATimestamp() - timeStepStarted >= 0.42) {
                            Robot.intake.intakeState = IntakeStates.intaking;
                        }

                        if (Timer.getFPGATimestamp() - timeStepStarted >= 8.97) {
                            Robot.intake.intakeState = IntakeStates.stationaryDeployed;
                            Robot.shooter.shooterState = ShooterStates.shooting;
                        }

                        break;
                    case 15:
                        Robot.drivebase.faceHub();
                        if (Robot.shooter.readyToShootInHub()) {
                            autoStep = 20;
                            Robot.teleop.timeIntakeShootingButtonPressed = Timer.getFPGATimestamp();

                        }
                        break;
                    case 20:
                        Robot.drivebase.faceHub();
                        Robot.hopper.hopperState = HopperStates.indexing;
                        Robot.kicker.kickerState = KickerStates.shooting;
                        Robot.intake.intakeState = IntakeStates.shooting;
                        break;
                }
                break;
            case Outpost_FullAcross_OneCycle:
                switch (autoStep) {
                    case 0:
                        autoToReturnTo = AutoRoutines.Outpost_FullAcross_OneCycle;
                        autoRoutine = AutoRoutines.OliviaAttemptGoOverBump;

                        Robot.shooter.shooterState = ShooterStates.stationary;
                        Robot.hopper.hopperState = HopperStates.stationary;
                        Robot.kicker.kickerState = KickerStates.stationary;
                        Robot.intake.intakeState = IntakeStates.stationaryDeployed;

                        Robot.drivebase.initPath("Outpost-fullAcross-OneCycle");

                        // path is over
                        timeStepStarted = Timer.getFPGATimestamp();
                        autoStep = 0;
                        break;

                    case 5:
                        if (Robot.drivebase.followLoadedPath()) {
                            timeStepStarted = Timer.getFPGATimestamp();
                            autoStep = 15;
                        }

                        if (Timer.getFPGATimestamp() - timeStepStarted >= 0.42) {
                            Robot.intake.intakeState = IntakeStates.intaking;
                        }

                        if (Timer.getFPGATimestamp() - timeStepStarted >= 9.41) {
                            Robot.intake.intakeState = IntakeStates.stationaryDeployed;
                            Robot.shooter.shooterState = ShooterStates.shooting;
                        }

                        break;
                    case 15:
                        Robot.drivebase.faceHub();
                        if (Robot.shooter.readyToShootInHub()) {
                            autoStep = 20;
                            Robot.teleop.timeIntakeShootingButtonPressed = Timer.getFPGATimestamp();

                        }
                        break;
                    case 20:
                        Robot.drivebase.faceHub();
                        Robot.hopper.hopperState = HopperStates.indexing;
                        Robot.kicker.kickerState = KickerStates.shooting;
                        Robot.intake.intakeState = IntakeStates.shooting;
                        break;
                }
                break;
            case Depot:
                switch (autoStep) {
                    case 0:
                        // if (Robot.onRed) {
                        // ifNoCameraAssumeRobotPos(new Pose2d(12.824, 2.489, new Rotation2d(180)));
                        // } else {
                        // ifNoCameraAssumeRobotPos(new Pose2d(3.677, 5.58, new Rotation2d(0)));
                        // }

                        Robot.shooter.shooterState = ShooterStates.stationary;
                        Robot.hopper.hopperState = HopperStates.stationary;
                        Robot.kicker.kickerState = KickerStates.stationary;
                        Robot.intake.intakeState = IntakeStates.stationaryDeployed;

                        Robot.drivebase.initPath("Depot");

                        // path is over
                        timeStepStarted = Timer.getFPGATimestamp();
                        autoStep = 0;
                        break;

                    case 5:
                        if (Robot.drivebase.followLoadedPath()) {
                            timeStepStarted = Timer.getFPGATimestamp();
                            autoStep = 15;
                        }

                        if (Timer.getFPGATimestamp() - timeStepStarted >= 1.5) {
                            Robot.intake.intakeState = IntakeStates.intaking;
                        }

                        if (Timer.getFPGATimestamp() - timeStepStarted >= 5) {
                            Robot.intake.intakeState = IntakeStates.stationaryDeployed;
                            Robot.shooter.shooterState = ShooterStates.shooting;
                        }

                        break;
                    case 15:
                        Robot.drivebase.faceHub();
                        if (Robot.shooter.readyToShootInHub()) {
                            autoStep = 20;
                            Robot.teleop.timeIntakeShootingButtonPressed = Timer.getFPGATimestamp();

                        }
                        break;
                    case 20:
                        Robot.drivebase.faceHub();
                        Robot.hopper.hopperState = HopperStates.indexing;
                        Robot.kicker.kickerState = KickerStates.shooting;
                        Robot.intake.intakeState = IntakeStates.shooting;
                        break;
                }
                break;
            case Depot_Depot_Outpost:
                switch (autoStep) {
                    case 0:
                        // if (Robot.onRed) {
                        // ifNoCameraAssumeRobotPos(new Pose2d(12.824, 2.489, new Rotation2d(180)));
                        // } else {
                        // ifNoCameraAssumeRobotPos(new Pose2d(3.677, 5.58, new Rotation2d(0)));
                        // }

                        Robot.shooter.shooterState = ShooterStates.stationary;
                        Robot.hopper.hopperState = HopperStates.stationary;
                        Robot.kicker.kickerState = KickerStates.stationary;
                        Robot.intake.intakeState = IntakeStates.stationaryDeployed;

                        Robot.drivebase.initPath("Depot-depot-outpost");

                        // path is over
                        timeStepStarted = Timer.getFPGATimestamp();
                        autoStep = 5;
                        break;

                    case 5:
                        if (Robot.drivebase.followLoadedPath()) {
                            timeStepStarted = Timer.getFPGATimestamp();
                            autoStep = 15;
                        }

                        if (Timer.getFPGATimestamp() - timeStepStarted >= 1.4) {
                            Robot.intake.intakeState = IntakeStates.intaking;
                        }

                        if (Timer.getFPGATimestamp() - timeStepStarted >= 11.3) {
                            Robot.intake.intakeState = IntakeStates.stationaryDeployed;
                            Robot.shooter.shooterState = ShooterStates.shooting;
                        }

                        break;
                    case 15:
                        Robot.drivebase.faceHub();
                        if (Robot.shooter.readyToShootInHub()) {
                            autoStep = 20;
                            Robot.teleop.timeIntakeShootingButtonPressed = Timer.getFPGATimestamp();

                        }
                        break;
                    case 20:
                        Robot.drivebase.faceHub();
                        Robot.hopper.hopperState = HopperStates.indexing;
                        Robot.kicker.kickerState = KickerStates.shooting;
                        Robot.intake.intakeState = IntakeStates.shooting;
                        break;
                }
                break;
            case Outpost:
                switch (autoStep) {
                    case 0:
                        // if (Robot.onRed) {
                        // ifNoCameraAssumeRobotPos(new Pose2d(12.837, 5.636, new Rotation2d(180)));
                        // } else {
                        // ifNoCameraAssumeRobotPos(new Pose2d(3.677, 2.644, new Rotation2d(0)));
                        // }

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
            case OliviaAttemptGoOverBump:
                if (Robot.isReal()) {
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
                                Robot.drivebase.driveFacingHeading(new Translation2d(-1.5, 0), new Rotation2d(Math.PI),
                                        true);
                            } else {
                                Robot.drivebase.driveFacingHeading(new Translation2d(1.5, 0), new Rotation2d(0), true);
                            }
                            if (Robot.drivebase.yagslDrive.getPitch()
                                    .getDegrees() > Settings.AutoSettings.Thresholds.autoDetectedBumpPitchTHold
                                    || Timer.getFPGATimestamp() - timeStepStarted >= 1) {
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
                            if (Robot.drivebase.yagslDrive.getPitch()
                                    .getDegrees() < -Settings.AutoSettings.Thresholds.autoDetectedBumpPitchTHold
                                    || Timer.getFPGATimestamp() - timeStepStarted >= 2) {
                                bumpTholdCounter++;
                            } else {
                                bumpTholdCounter = 0;
                            }
                            if (Robot.onRed) {
                                Robot.drivebase.driveFacingHeading(new Translation2d(-1.5, 0), new Rotation2d(Math.PI),
                                        true);
                            } else {
                                Robot.drivebase.driveFacingHeading(new Translation2d(1.5, 0), new Rotation2d(0), true);
                            }

                            if (bumpTholdCounter > Settings.AutoSettings.Thresholds.autoDetectedBumpPitchCount) {
                                bumpTholdCounter = 0;
                                autoStep = 20;
                            }
                            break;
                        case 20:
                            if (Math.abs(Robot.drivebase.yagslDrive.getPitch()
                                    .getDegrees()) < Settings.AutoSettings.Thresholds.detectedFlatTHold
                                    || Timer.getFPGATimestamp() - timeStepStarted >= 2) {
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
                                timeStepStarted = Timer.getFPGATimestamp();
                            }
                    }
                } else {
                    autoRoutine = autoToReturnTo;
                    autoStep = 5;
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
        if (Timer.getFPGATimestamp() - Robot.vision.timeATLastSeen > 20) {
            Robot.drivebase.yagslDrive.resetOdometry(autoStartPose);
        }
    }

}
