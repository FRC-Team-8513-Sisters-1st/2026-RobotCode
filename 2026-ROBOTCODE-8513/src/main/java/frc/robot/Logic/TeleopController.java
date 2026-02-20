package frc.robot.Logic;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Robot;
import frc.robot.Settings;
import frc.robot.Logic.Enums.HopperStates;
import frc.robot.Logic.Enums.IntakeStates;
import frc.robot.Logic.Enums.KickerStates;
import frc.robot.Logic.Enums.ShooterStates;

public class TeleopController {

    public Joystick driverXboxController = new Joystick(Settings.TeleopSettings.driverJoystickPort);
    public Joystick copilotXboxController = new Joystick(Settings.TeleopSettings.copilotJoystickPort);
    public Joystick manualJoystick = new Joystick(Settings.TeleopSettings.manualJoystickPort);

    public PIDController rJoystickController = new PIDController(0.1, 0, 0);

    public Rotation2d goalHeading = new Rotation2d();

    SlewRateLimiter xfilter = new SlewRateLimiter(4);
    SlewRateLimiter yfilter = new SlewRateLimiter(4);
    SlewRateLimiter rfilter = new SlewRateLimiter(4);

    public boolean shootingFacingHub = false;
    public boolean useSpecialNewRotation = true;
    public boolean autoShooting = true;

    public double shooterButtonTime;

    public Pose2d copilotShuttlePosition = Settings.FieldInfo.ShuttlingPositions.neutralZone;

    public double xV;
    public double yV;
    public double rV;

    public void initTele() {
        Robot.shooter.initShooter();
        Robot.dashboard.getPIDValues();

        // consume all buttons pressed
        for (int i = 0; i < 14; i++) {
            driverXboxController.getRawButtonPressed(i);
            copilotXboxController.getRawButtonPressed(i);
        }

    }

    public double xSpeedJoystick;
    public double ySpeedJoystick;

    public void driveTele() {

        // drivebase controls
        xSpeedJoystick = -driverXboxController.getRawAxis(Settings.TeleopSettings.forwardBackwardsAxis); // forward
                                                                                                         // back
        if (xSpeedJoystick < Settings.TeleopSettings.joystickDeadband
                && xSpeedJoystick > -Settings.TeleopSettings.joystickDeadband) {
            xSpeedJoystick = 0;
        }
        xSpeedJoystick = xfilter.calculate(xSpeedJoystick);

        ySpeedJoystick = -driverXboxController.getRawAxis(Settings.TeleopSettings.leftRightAxis); // left right
        if (ySpeedJoystick < Settings.TeleopSettings.joystickDeadband
                && ySpeedJoystick > -Settings.TeleopSettings.joystickDeadband) {
            ySpeedJoystick = 0;

        }
        ySpeedJoystick = yfilter.calculate(ySpeedJoystick);

        double rSpeedJoystick = -driverXboxController.getRawAxis(Settings.TeleopSettings.rotAxis); // left right 2 at
                                                                                                   // home, 4 on xbox
        double rySpeedJoystick = driverXboxController.getRawAxis(Settings.TeleopSettings.RleftRightAxis);
        double rxSpeedJoystick = driverXboxController.getRawAxis(Settings.TeleopSettings.RforwardBackwardsAxis);

        if (rSpeedJoystick < Settings.TeleopSettings.joystickDeadband
                && rSpeedJoystick > -Settings.TeleopSettings.joystickDeadband) {
            rSpeedJoystick = 0;
        }

        rSpeedJoystick = rfilter.calculate(rSpeedJoystick);

        // cube the joystick values for smoother control
        double xInput = Math.pow(xSpeedJoystick, 1);
        double yInput = Math.pow(ySpeedJoystick, 1);
        double rInput = Math.pow(rSpeedJoystick, 3);

        // if facing hub button, slow speed to 0.3
        double robotV = Robot.drivebase.getRobotVelocityHypotenuse();
        if (shootingFacingHub) {
            if (Robot.onRed) {
                xV = -(xInput * Robot.drivebase.yagslDrive.getMaximumChassisVelocity() * 0.3);
                yV = -(yInput * Robot.drivebase.yagslDrive.getMaximumChassisVelocity() * 0.3);
                rV = rInput * Robot.drivebase.yagslDrive.getMaximumChassisAngularVelocity();
            } else {
                xV = xInput * Robot.drivebase.yagslDrive.getMaximumChassisVelocity() * 0.3;
                yV = yInput * Robot.drivebase.yagslDrive.getMaximumChassisVelocity() * 0.3;
                rV = rInput * Robot.drivebase.yagslDrive.getMaximumChassisAngularVelocity();
            }
            // if driving over bump, do the bump speed calculation and lower speed if we are
            // going over
        } else if (Robot.drivebase.drivingOverBump() && robotV > Settings.PhysicalRobotValues.bumpMaxVelocity) {
            if (Robot.onRed) {
                xV = -(xInput * Robot.drivebase.yagslDrive.getMaximumChassisVelocity()
                        * (Settings.PhysicalRobotValues.bumpMaxVelocity / robotV));
                yV = -(yInput * Robot.drivebase.yagslDrive.getMaximumChassisVelocity()
                        * (Settings.PhysicalRobotValues.bumpMaxVelocity / robotV));
                rV = rInput * Robot.drivebase.yagslDrive.getMaximumChassisAngularVelocity();
            } else {
                xV = xInput * Robot.drivebase.yagslDrive.getMaximumChassisVelocity()
                        * (Settings.PhysicalRobotValues.bumpMaxVelocity / robotV);
                yV = yInput * Robot.drivebase.yagslDrive.getMaximumChassisVelocity()
                        * (Settings.PhysicalRobotValues.bumpMaxVelocity / robotV);
                rV = rInput * Robot.drivebase.yagslDrive.getMaximumChassisAngularVelocity();
            }
            // default driving
        } else {
            if (Robot.onRed) {
                xV = -(xInput * Robot.drivebase.yagslDrive.getMaximumChassisVelocity());
                yV = -(yInput * Robot.drivebase.yagslDrive.getMaximumChassisVelocity());
                rV = rInput * Robot.drivebase.yagslDrive.getMaximumChassisAngularVelocity();
            } else {
                xV = xInput * Robot.drivebase.yagslDrive.getMaximumChassisVelocity();
                yV = yInput * Robot.drivebase.yagslDrive.getMaximumChassisVelocity();
                rV = rInput * Robot.drivebase.yagslDrive.getMaximumChassisAngularVelocity();
            }
        }

        // special rotation
        if (Math.hypot(rySpeedJoystick,
                rxSpeedJoystick) < Settings.TeleopSettings.specialRotationJoystickDeadband) {
            Robot.drivebase.goalHeading = Robot.drivebase.goalHeading;
        } else {
            Rotation2d rotation;
            if (Robot.onRed) {
                rotation = new Rotation2d(rySpeedJoystick, rxSpeedJoystick);
            } else {
                rotation = new Rotation2d(-rySpeedJoystick, -rxSpeedJoystick);
            }
            Robot.drivebase.goalHeading = rotation;
        }

        // if face hub button is pressed, rotate to hub then set the kicker and hopper
        // states
        if (driverXboxController.getRawButton(Settings.TeleopSettings.ButtonIDs.faceGoal)) {
            if (Robot.onRed) {
                if (Robot.drivebase.yagslDrive.getPose().getX() > 11.8) {
                    shootingFacingHub = true;
                    Robot.drivebase.yagslDrive.drive(new Translation2d(xV, yV), Robot.drivebase.getPowerToFaceHub(),
                            true,
                            false);
                } else {
                    Robot.drivebase.driveFacingPose(new Translation2d(xV, yV), copilotShuttlePosition, true);
                }
            } else {
                if (Robot.drivebase.yagslDrive.getPose().getX() < 4.66) {
                    shootingFacingHub = true;
                    Robot.drivebase.yagslDrive.drive(new Translation2d(xV, yV), Robot.drivebase.getPowerToFaceHub(),
                            true,
                            false);
                } else {
                    Robot.drivebase.driveFacingPose(new Translation2d(xV, yV), copilotShuttlePosition, true);
                }
            }
            if (Robot.shooter.readyToShoot() && autoShooting) {
                Robot.kicker.kickerState = KickerStates.shooting;
                Robot.hopper.hopperState = HopperStates.indexing;
            } else {
                Robot.kicker.kickerState = KickerStates.stationary;
                Robot.hopper.hopperState = HopperStates.stationary;
            }
            // if straighten bump pressed, check rotation and switch to nearest 0 or 180
        } else if (driverXboxController.getRawButton(Settings.TeleopSettings.ButtonIDs.straightenBump)) {
            if (Math.abs(Robot.drivebase.yagslDrive.getOdometryHeading().getDegrees()) <= 90) {
                Robot.drivebase.yagslDrive.drive(new Translation2d(xV, yV),
                        Robot.drivebase.getPowerToReachRotation(new Rotation2d(0)), true,
                        false);
                Robot.drivebase.goalHeading = new Rotation2d(0);
            } else {
                Robot.drivebase.yagslDrive.drive(new Translation2d(xV, yV),
                        Robot.drivebase.getPowerToReachRotation(new Rotation2d(Math.PI)), true,
                        false);
                Robot.drivebase.goalHeading = new Rotation2d(Math.PI);
            }
            // otherwise, use the normal drive
        } else if (driverXboxController
                .getRawButton(Settings.TeleopSettings.ButtonIDs.snakeMode)) {
            if (Math.abs(xSpeedJoystick) < Settings.TeleopSettings.joystickDeadband
                    && Math.abs(ySpeedJoystick) < Settings.TeleopSettings.joystickDeadband) {
                Robot.drivebase.driveFacingHeading(new Translation2d(xV, yV), Robot.drivebase.goalHeading, true);
            } else {
                Rotation2d snakeModeRotation = new Translation2d(xV, yV).getAngle();
                Robot.drivebase.driveFacingHeading(new Translation2d(xV, yV), snakeModeRotation, true);
            }

        } else {
            shootingFacingHub = false;
            Robot.drivebase.driveFacingHeading(new Translation2d(xV, yV), Robot.drivebase.goalHeading, true);
        }

        buttonControls();
        Robot.dashboard.copilotField2d.setRobotPose(copilotShuttlePosition.getX(), copilotShuttlePosition.getY(), copilotShuttlePosition.getRotation());


        // Subsystem set motor power
        Robot.shooter.setMotorPower();
        Robot.intake.setMotorPower();
        Robot.kicker.setMotorPower();
        Robot.hopper.setMotorPower();
    }

    // button pressed --> state machine
    public void buttonControls() {
        // intake controls
        boolean intakeButtonPressed = Robot.teleop.driverXboxController
                .getRawButtonPressed(Settings.TeleopSettings.ButtonIDs.intake);
        boolean stopIntakeButtonPressed = Robot.teleop.driverXboxController
                .getRawButtonPressed(Settings.TeleopSettings.ButtonIDs.stopIntake);
        if (Robot.intake.intakeState == IntakeStates.stowed && intakeButtonPressed) {
            // if the robot is stowed
            Robot.intake.intakeState = IntakeStates.intaking;
        } else if ((Robot.intake.intakeState == IntakeStates.intaking
                || Robot.intake.intakeState == IntakeStates.stationaryDeployed)
                && intakeButtonPressed) {
            // if the robot is intaking or stationary
            Robot.intake.intakeState = IntakeStates.stowed;
        } else if (Robot.intake.intakeState == IntakeStates.intaking
                && stopIntakeButtonPressed) {
            // turn the wheels off while deployed
            Robot.intake.intakeState = IntakeStates.stationaryDeployed;
        } else if (Robot.intake.intakeState == IntakeStates.stationaryDeployed
                && stopIntakeButtonPressed) {
            // turn the wheels on while deployed
            Robot.intake.intakeState = IntakeStates.intaking;
        }

        // TEMPORARY: reset the intake motors
        if (driverXboxController.getRawButtonPressed(8)) {
            Settings.resetTalon(Robot.intake.intakeMotorLeftLeader);
            Settings.resetTalon(Robot.intake.intakeMotorRightFollower);
        }

        // COPILOT CONTROLS
        // adjustment for shooter hood angle
        if (copilotXboxController.getRawButtonPressed(Settings.TeleopSettings.ButtonIDs.increaseShotDistance)) {
            Robot.shooter.shotDistanceFudgeFactor += Settings.ShooterSettings.hoodAngleFudgeFactor;
        } else if (copilotXboxController.getRawButtonPressed(Settings.TeleopSettings.ButtonIDs.decreaseShotDistance)) {
            Robot.shooter.shotDistanceFudgeFactor -= Settings.ShooterSettings.hoodAngleFudgeFactor;
        }

        // intake Fudge Factor
        if (copilotXboxController.getRawButtonPressed(Settings.TeleopSettings.ButtonIDs.heightenIntake)) {
            Robot.intake.intakeFudgeFactor -= Settings.IntakeSettings.intakeFudgeFactor;
        } else if (copilotXboxController.getRawButtonPressed(Settings.TeleopSettings.ButtonIDs.lowerIntake)) {
            Robot.intake.intakeFudgeFactor += Settings.IntakeSettings.intakeFudgeFactor;
        }

        // adjustment for drivebase goal aim fudge factor
        if (copilotXboxController.getRawButtonPressed(Settings.TeleopSettings.ButtonIDs.moveScorePoseRight)) {
            Robot.drivebase.aimFudgeFactor += Settings.ShooterSettings.aimFudgeFactor;
        } else if (copilotXboxController.getRawButtonPressed(Settings.TeleopSettings.ButtonIDs.moveScorePoseLeft)) {
            Robot.drivebase.aimFudgeFactor -= Settings.ShooterSettings.aimFudgeFactor;
        }

        // Intake copilot emergency controls
        boolean copilotEmergencyIntakeButtonPressed = Robot.teleop.copilotXboxController
                .getRawButtonPressed(Settings.TeleopSettings.ButtonIDs.emergencyIntake);
        if (Robot.intake.intakeState == IntakeStates.stowed && copilotEmergencyIntakeButtonPressed) {
            // if the robot is stowed
            Robot.intake.intakeState = IntakeStates.intaking;
        } else if ((Robot.intake.intakeState == IntakeStates.intaking
                || Robot.intake.intakeState == IntakeStates.stationaryDeployed)
                && copilotEmergencyIntakeButtonPressed) {
            // if the robot is intaking or stationary
            Robot.intake.intakeState = IntakeStates.stowed;
        }

        // shooter controls
        boolean shootButtonPressed = copilotXboxController
                .getRawButtonPressed(Settings.TeleopSettings.ButtonIDs.manualShoot);
        if (autoShooting && shootButtonPressed && Robot.shooter.shooterState == ShooterStates.stationary) {
            Robot.shooter.shooterState = ShooterStates.shooting;
            Robot.kicker.kickerState = KickerStates.shooting;
            Robot.hopper.hopperState = HopperStates.indexing;
        } else if (autoShooting && shootButtonPressed && Robot.shooter.shooterState == ShooterStates.shooting) {
            Robot.shooter.shooterState = ShooterStates.stationary;
            Robot.kicker.kickerState = KickerStates.stationary;
            Robot.hopper.hopperState = HopperStates.stationary;
        }

        // kicker controls
        boolean indexerKicker = copilotXboxController.getRawButtonPressed(Settings.TeleopSettings.ButtonIDs.kicker);
        if (indexerKicker && Robot.kicker.kickerState == KickerStates.stationary) {
            Robot.kicker.kickerState = KickerStates.shooting;
            Robot.hopper.hopperState = HopperStates.indexing;
        } else if (indexerKicker && Robot.kicker.kickerState == KickerStates.shooting) {
            Robot.kicker.kickerState = KickerStates.stationary;
            Robot.hopper.hopperState = HopperStates.stationary;
        }

        // turns on/off auto shoot
        boolean autoShootButtonPressed = copilotXboxController
                .getRawButtonPressed(Settings.TeleopSettings.ButtonIDs.toggleAutoShoot);
        if (autoShootButtonPressed) {
            autoShooting = false;
        } else {
            autoShooting = true;
        }

        // indexer
        if (copilotXboxController
                .getRawButton(Settings.TeleopSettings.ButtonIDs.startIndexer)) {
            Robot.hopper.hopperState = HopperStates.indexing;
        } else if (copilotXboxController
                .getRawButton(Settings.TeleopSettings.ButtonIDs.stopIndexer)) {
            Robot.hopper.hopperState = HopperStates.stationary;
        } else if (copilotXboxController
                .getRawButton(Settings.TeleopSettings.ButtonIDs.reverseIndexer)) {
            Robot.hopper.hopperState = HopperStates.unjam;
        }

        // shuttle position buttons
        boolean redDepotTrenchButtonPressed = copilotXboxController
                .getRawButtonPressed(Settings.TeleopSettings.ButtonIDs.redDepotTrenchButton);
        boolean blueDepotTrenchButtonPressed = copilotXboxController
                .getRawButtonPressed(Settings.TeleopSettings.ButtonIDs.blueDepotTrenchButton);
        boolean redOutpostTrenchButtonPressed = copilotXboxController
                .getRawButtonPressed(Settings.TeleopSettings.ButtonIDs.redOutpostTrenchButton);
        boolean blueOutpostTrenchButtonPressed = copilotXboxController
                .getRawButtonPressed(Settings.TeleopSettings.ButtonIDs.blueOutpostTrenchButton);
        boolean nuetralZoneButtonPressed = copilotXboxController
                .getRawButtonPressed(Settings.TeleopSettings.ButtonIDs.nuetralZoneButton);
        if (redDepotTrenchButtonPressed) {
            copilotShuttlePosition = Settings.FieldInfo.ShuttlingPositions.redDepotTrench;
        } else if (blueDepotTrenchButtonPressed) {
            copilotShuttlePosition = Settings.FieldInfo.ShuttlingPositions.blueDepotTrench;
        } else if (redOutpostTrenchButtonPressed) {
            copilotShuttlePosition = Settings.FieldInfo.ShuttlingPositions.redOutpostTrench;
        } else if (blueOutpostTrenchButtonPressed) {
            copilotShuttlePosition = Settings.FieldInfo.ShuttlingPositions.blueOutpostTrench;
        } else if (nuetralZoneButtonPressed) {
            copilotShuttlePosition = Settings.FieldInfo.ShuttlingPositions.neutralZone;
        }
    }

}
