package frc.robot.Logic;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
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

    public void driveTele() {

        // drivebase controls
        double xSpeedJoystick = -driverXboxController.getRawAxis(Settings.TeleopSettings.forwardBackwardsAxis); // forward
                                                                                                                // back
        if (xSpeedJoystick < Settings.TeleopSettings.joystickDeadband
                && xSpeedJoystick > -Settings.TeleopSettings.joystickDeadband) {
            xSpeedJoystick = 0;
        }
        xSpeedJoystick = xfilter.calculate(xSpeedJoystick);

        double ySpeedJoystick = -driverXboxController.getRawAxis(Settings.TeleopSettings.leftRightAxis); // left right
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

        double multFactor;

        if (shootingFacingHub) {
            multFactor = 0.3;
            if (Robot.onRed) {
                xV = -(xInput * Robot.drivebase.yagslDrive.getMaximumChassisVelocity() * multFactor);
                yV = -(yInput * Robot.drivebase.yagslDrive.getMaximumChassisVelocity() * multFactor);
                rV = rInput * Robot.drivebase.yagslDrive.getMaximumChassisAngularVelocity();
            } else {
                xV = xInput * Robot.drivebase.yagslDrive.getMaximumChassisVelocity() * multFactor;
                yV = yInput * Robot.drivebase.yagslDrive.getMaximumChassisVelocity() * multFactor;
                rV = rInput * Robot.drivebase.yagslDrive.getMaximumChassisAngularVelocity();
            }
        } else {
            multFactor = Robot.drivebase.drivingOverBump();
            if (Robot.onRed) {
                xV = -(xInput * Robot.drivebase.yagslDrive.getMaximumChassisVelocity() * multFactor);
                yV = -(yInput * Robot.drivebase.yagslDrive.getMaximumChassisVelocity() * multFactor);
                rV = rInput * Robot.drivebase.yagslDrive.getMaximumChassisAngularVelocity();
            } else {
                xV = xInput * Robot.drivebase.yagslDrive.getMaximumChassisVelocity() * multFactor;
                yV = yInput * Robot.drivebase.yagslDrive.getMaximumChassisVelocity() * multFactor;
                rV = rInput * Robot.drivebase.yagslDrive.getMaximumChassisAngularVelocity();
            }
        }

        // drive/face hub
        if (useSpecialNewRotation) {
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
        } else {
            Robot.drivebase.goalHeading = new Rotation2d(rV);
        }

        if (driverXboxController.getRawButton(Settings.TeleopSettings.ButtonIDs.faceHub)) {
            shootingFacingHub = true;
            Robot.drivebase.yagslDrive.drive(new Translation2d(xV, yV), Robot.drivebase.getPowerToFaceHub(), true,
                    false);
            if (Robot.shooter.readyToShoot() && autoShooting) {
                Robot.kicker.kickerState = KickerStates.shooting;
                Robot.hopper.hopperState = HopperStates.indexing;
            } else {
                Robot.kicker.kickerState = KickerStates.stationary;
                Robot.hopper.hopperState = HopperStates.stationary;
            }
        } else {
            shootingFacingHub = false;
            if (useSpecialNewRotation) {
                Robot.drivebase.driveFacingHeading(new Translation2d(xV, yV), Robot.drivebase.goalHeading, true);
            } else {
                Robot.drivebase.yagslDrive.drive(new Translation2d(xV, yV), rV, true, false);
            }
        }

        buttonControls();

        // Subsystem set motor power
        Robot.shooter.setMotorPower();
        Robot.intake.setMotorPower();
        Robot.kicker.setMotorPower();
        Robot.hopper.setMotorPower();
    }

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


        // COPILOT CONTROLS
        // adjustment for shooter hood angle
        if (copilotXboxController.getRawButtonPressed(Settings.TeleopSettings.ButtonIDs.increaseAngle)) {
            Robot.shooter.angleFudgeFactor += Settings.ShooterSettings.shooterFudgeFactor;
        } else if (copilotXboxController.getRawButtonPressed(Settings.TeleopSettings.ButtonIDs.decreaseAngle)) {
            Robot.shooter.angleFudgeFactor -= Settings.ShooterSettings.shooterFudgeFactor;
        }

        // intake Fudge Factor
        if (copilotXboxController.getRawButtonPressed(Settings.TeleopSettings.ButtonIDs.heightenIntake)) {
            Robot.intake.intakeFudgeFactor -= Settings.IntakeSettings.intakeFudgeFactor;
        } else if (copilotXboxController.getRawButtonPressed(Settings.TeleopSettings.ButtonIDs.lowerIntake)) {
            Robot.intake.intakeFudgeFactor += Settings.IntakeSettings.intakeFudgeFactor;
        }

        // adjustment for drivebase goal aim fudge factor
        if (copilotXboxController.getRawButtonPressed(Settings.TeleopSettings.ButtonIDs.moveScorePoseRight)) {
            Robot.drivebase.aimFudgeFactor += Settings.ShooterSettings.angleFudgeFactor;
        } else if (copilotXboxController.getRawButtonPressed(Settings.TeleopSettings.ButtonIDs.moveScorePoseLeft)) {
            Robot.drivebase.aimFudgeFactor -= Settings.ShooterSettings.angleFudgeFactor;
        }

        // Intake copilot emergency controls
        boolean copilotEmergencyIntakeButtonPressed = Robot.teleop.driverXboxController
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

        // shooter/kicker controls
        boolean shootButtonPressed = copilotXboxController.getRawButtonPressed(Settings.TeleopSettings.ButtonIDs.manualShoot);
        if (shootButtonPressed && Robot.shooter.shooterState == ShooterStates.stationary) {
            Robot.shooter.shooterState = ShooterStates.shooting;
            Robot.kicker.kickerState = KickerStates.shooting;
            Robot.hopper.hopperState = HopperStates.indexing;
        } else if (shootButtonPressed && Robot.shooter.shooterState == ShooterStates.shooting) {
            Robot.shooter.shooterState = ShooterStates.stationary;
            Robot.kicker.kickerState = KickerStates.stationary;
            Robot.hopper.hopperState = HopperStates.stationary;
        }

        if (driverXboxController.getRawButtonPressed(8)) {
            Settings.resetTalon(Robot.intake.intakeMotorLeftLeader);
            Settings.resetTalon(Robot.intake.intakeMotorRightFollower);

        }

        boolean autoShootButtonPressed = copilotXboxController.getRawButtonPressed(Settings.TeleopSettings.ButtonIDs.manualShoot);
        if (autoShootButtonPressed) {
            autoShooting = false;
        }
    }

}
