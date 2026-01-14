package frc.robot.Logic;

import java.util.Set;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Robot;

public class TeleopController {

    Joystick driverJoystick = new Joystick(Settings.TeleopSettings.DriverJoystick.port);
    public boolean fieldRelative = true;

    public Rotation2d goalHeading = new Rotation2d();
    public PIDController rotationPidController = new PIDController(Settings.DrivebaseSettings.RotationPIDConstants.kP,
            Settings.DrivebaseSettings.RotationPIDConstants.kI,
            Settings.DrivebaseSettings.RotationPIDConstants.kD);

    public TeleopController() {

    }

    public void teleopLoop() {
        double xJoystickValule = driverJoystick.getRawAxis(Settings.TeleopSettings.DriverJoystick.xAxis);
        double yJoystickValule = driverJoystick.getRawAxis(Settings.TeleopSettings.DriverJoystick.yAxis);
        double rJoystickValule = -driverJoystick.getRawAxis(Settings.TeleopSettings.DriverJoystick.rAxis);

        if (Math.abs(xJoystickValule) < Settings.TeleopSettings.DriverJoystick.deadband) {
            xJoystickValule = 0;
        }
        if (Math.abs(yJoystickValule) < Settings.TeleopSettings.DriverJoystick.deadband) {
            yJoystickValule = 0;
        }
        if (Math.abs(rJoystickValule) < Settings.TeleopSettings.DriverJoystick.deadband) {
            rJoystickValule = 0;
        }

        if (Robot.onRed == false) {
            xJoystickValule *= -1;
            yJoystickValule *= -1;
        }

        Translation2d robotVelocity = new Translation2d(xJoystickValule * Settings.DrivebaseSettings.maxVelocityMPS,
                yJoystickValule * Settings.DrivebaseSettings.maxVelocityMPS);

        if (!(Math.abs(rJoystickValule) < Settings.TeleopSettings.DriverJoystick.deadband)) {
            goalHeading = Robot.drivebase.yagslDrive.getOdometryHeading()
                    .plus(new Rotation2d(
                            rJoystickValule * Settings.TeleopSettings.DriverJoystick.rotationalJoystickSensitivity));

        }

        double angleError = Robot.drivebase.yagslDrive.getOdometryHeading().minus(goalHeading).getDegrees();
        double rotationCorrection = rotationPidController.calculate(angleError, 0);

        Robot.drivebase.yagslDrive.drive(robotVelocity,
                rotationCorrection,
                fieldRelative,
                false);
    }

}
