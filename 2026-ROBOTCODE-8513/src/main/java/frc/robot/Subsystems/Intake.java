package frc.robot.Subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Robot;
import frc.robot.Settings;
import frc.robot.Logic.Enums.IntakeStates;

public class Intake {
    public IntakeStates intakeState = IntakeStates.stationaryDeployed;

    public TalonFX intakeMotorLeft = new TalonFX(31);
    public TalonFX intakeMotorRight = new TalonFX(32);
    public TalonFX intakeDeployMotor = new TalonFX(33);

    public PIDController intakeMotorController = new PIDController(0.1, 0, 0);

    public boolean useManualIntakeControl = false;

    public Intake() {
    }

    public void setMotorPower() {

        if (intakeState == IntakeStates.intaking) {
            // deploy intake
            intakeDeployMotor.set(deployPower(Settings.IntakeSettings.deployPosition));

            // intake wheels on
            intakeMotorLeft.set(1);
            intakeMotorRight.set(-1);

        } else if (intakeState == IntakeStates.stowed) {
            // stow intake
            intakeDeployMotor.set(deployPower(Settings.IntakeSettings.stowPosition));

            // intake wheels off
            intakeMotorLeft.set(0);
            intakeMotorRight.set(0);

        } else if (intakeState == IntakeStates.outtaking) {
            // deploy intake
            intakeDeployMotor.set(deployPower(Settings.IntakeSettings.deployPosition));

            // intake wheels on
            intakeMotorLeft.set(-1);
            intakeMotorRight.set(1);
        } else if (intakeState == IntakeStates.stationaryDeployed) {
            // deploy intake
            intakeDeployMotor.set(deployPower(Settings.IntakeSettings.deployPosition));

            // intake wheels off
            intakeMotorLeft.set(0);
            intakeMotorRight.set(0);
        }

        // TEMPORARY: manual joystick control for intake
        if (useManualIntakeControl) {
            double intakeJoystickValue = Robot.teleop.manualJoystick.getRawAxis(5) * 0.2;

            if (Math.abs(intakeJoystickValue) > Settings.TeleopSettings.joystickDeadband) {
                intakeDeployMotor.set(intakeJoystickValue);
                double currentPosition = intakeDeployMotor.getPosition().getValueAsDouble();
                intakeMotorController.setSetpoint(currentPosition);
            } else {
                double power = intakeMotorController.calculate(intakeDeployMotor.getPosition().getValueAsDouble());
                intakeDeployMotor.set(power);
            }

            if (Robot.teleop.manualJoystick.getRawButtonPressed(8)) {
                intakeDeployMotor.setPosition(0);
            }
        }
    }

    public double deployPower(double targetPosition) {
        double currentPosition = intakeDeployMotor.getPosition().getValueAsDouble();
        double outputPower = intakeMotorController.calculate(currentPosition, targetPosition);
        return outputPower;
    }
}
