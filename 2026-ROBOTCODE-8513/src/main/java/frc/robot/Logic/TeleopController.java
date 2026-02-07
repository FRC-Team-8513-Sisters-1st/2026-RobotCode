package frc.robot.Logic;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Robot;
import frc.robot.Settings;
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

    public void initTele() {
        Robot.shooter.initShooter();

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
        if (rSpeedJoystick < Settings.TeleopSettings.joystickDeadband
                && rSpeedJoystick > -Settings.TeleopSettings.joystickDeadband) {
            rSpeedJoystick = 0;

        }
        rSpeedJoystick = rfilter.calculate(rSpeedJoystick);

        // cube the joystick values for smoother control
        double xInput = Math.pow(xSpeedJoystick, 3);
        double yInput = Math.pow(ySpeedJoystick, 3);
        double rInput = Math.pow(rSpeedJoystick, 3);

        double xV;
        double yV;
        double rV;

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
        } else {
            if (Robot.onRed) {
                xV = -(xInput * Robot.drivebase.yagslDrive.getMaximumChassisVelocity() * 0.8);
                yV = -(yInput * Robot.drivebase.yagslDrive.getMaximumChassisVelocity() * 0.8);
                rV = rInput * Robot.drivebase.yagslDrive.getMaximumChassisAngularVelocity();
            } else {
                xV = xInput * Robot.drivebase.yagslDrive.getMaximumChassisVelocity() * 0.8;
                yV = yInput * Robot.drivebase.yagslDrive.getMaximumChassisVelocity() * 0.8;
                rV = rInput * Robot.drivebase.yagslDrive.getMaximumChassisAngularVelocity();
            }
        }

        // drive/face hub
        Robot.drivebase.goalHeading = new Rotation2d(rV);
        if (driverXboxController.getRawButton(Settings.TeleopSettings.ButtonIDs.faceHub)) {
            shootingFacingHub = true;
            Robot.drivebase.yagslDrive.drive(new Translation2d(xV, yV), Robot.drivebase.getRotationToHub(), true,
                    false);
        } else {
            shootingFacingHub = false;
            Robot.drivebase.yagslDrive.drive(new Translation2d(xV, yV), rV, true, false);
        }

        // intake controls
        boolean intakeButtonPressed = Robot.teleop.driverXboxController.getRawButtonPressed(Settings.TeleopSettings.ButtonIDs.intake);
        if (Robot.intake.intakeState == IntakeStates.stowed && intakeButtonPressed) {
            // if the robot is stowed
            Robot.intake.intakeState = IntakeStates.intaking;
        } else if ((Robot.intake.intakeState == IntakeStates.intaking
                || Robot.intake.intakeState == IntakeStates.stationaryDeployed)
                && intakeButtonPressed) {
            // if the robot is intaking or stationary
            Robot.intake.intakeState = IntakeStates.stowed;
        } else if (Robot.intake.intakeState == IntakeStates.intaking
                && intakeButtonPressed) {
            // turn the wheels off while deployed
            Robot.intake.intakeState = IntakeStates.stationaryDeployed;
        }

        // shooter/kicker controls
        boolean shootButtonPressed = driverXboxController.getRawButtonPressed(Settings.TeleopSettings.ButtonIDs.shoot);
        if (shootButtonPressed && Robot.shooter.shooterState == ShooterStates.stationary) {
            Robot.shooter.shooterState = ShooterStates.shooting;
            Robot.kicker.kickerState = KickerStates.shooting;
        } else if (shootButtonPressed && Robot.shooter.shooterState == ShooterStates.shooting) {
            Robot.shooter.shooterState = ShooterStates.stationary;
            Robot.kicker.kickerState = KickerStates.stationary;
        }

        // COPILOT CONTROLS
        // adjustment for shooter hood angle
        if (copilotXboxController.getRawButton(Settings.TeleopSettings.ButtonIDs.increaseAngle)){
            Robot.shooter.angleFudgeFactor += Settings.ShooterSettings.shooterFudgeFactor;
        } else if (copilotXboxController.getRawButton(Settings.TeleopSettings.ButtonIDs.decreaseAngle)){
            Robot.shooter.angleFudgeFactor -= Settings.ShooterSettings.shooterFudgeFactor;
        }

        // adjustment for drivebase goal aim fudge factor
        if (copilotXboxController.getRawButton(Settings.TeleopSettings.ButtonIDs.moveScorePoseRight)){
            Robot.drivebase.aimFudgeFactor += Settings.ShooterSettings.angleFudgeFactor;
        } else if (copilotXboxController.getRawButton(Settings.TeleopSettings.ButtonIDs.moveScorePoseLeft)){
            Robot.drivebase.aimFudgeFactor -= Settings.ShooterSettings.angleFudgeFactor;
        } 

        // Subsystem set motor power
        Robot.shooter.setMotorPower();
        Robot.intake.setMotorPower();
        Robot.kicker.setMotorPower();

    }
}
