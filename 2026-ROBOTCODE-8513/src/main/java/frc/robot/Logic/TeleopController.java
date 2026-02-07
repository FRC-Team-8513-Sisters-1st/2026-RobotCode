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
        // shooter/kicker controls
        boolean shootButtonPressed = driverXboxController.getRawButtonPressed(Settings.TeleopSettings.ButtonIDs.shoot);
        if (shootButtonPressed && Robot.shooter.shooterState == ShooterStates.stationary) {
            Robot.shooter.shooterState = ShooterStates.shooting;
            Robot.kicker.kickerState = KickerStates.shooting;
        } else if (shootButtonPressed && Robot.shooter.shooterState == ShooterStates.shooting) {
            Robot.shooter.shooterState = ShooterStates.stationary;
            Robot.kicker.kickerState = KickerStates.stationary;
        }

        
        // Subsystem set motor power
        Robot.shooter.setMotorPower();
        Robot.kicker.setMotorPower();

    }
}
