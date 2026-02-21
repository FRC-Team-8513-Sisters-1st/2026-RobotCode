package frc.robot.Subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Robot;
import frc.robot.Settings;
import frc.robot.Logic.Enums.HopperStates;
import frc.robot.Logic.Enums.IntakeStates;
import frc.robot.Logic.Enums.KickerStates;

public class Intake {
    public IntakeStates intakeState = IntakeStates.stationaryDeployed;

    // class member variable
    final DutyCycleOut m_request = new DutyCycleOut(0);

    public TalonFX intakeMotorLeftLeader = new TalonFX(32);
    public TalonFX intakeMotorRightFollower = new TalonFX(31);
    public TalonFX intakeDeployMotor = new TalonFX(33);

    public PIDController intakeMotorController = new PIDController(0.1, 0, 0);

    public boolean useManualIntakeControl = false;
    public double intakeFudgeFactor = 0;

    public Intake() {
        intakeMotorRightFollower
                .setControl(new Follower(intakeMotorLeftLeader.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    public void setMotorPower() {

        if (intakeState == IntakeStates.intaking) {
            // deploy intake
            intakeDeployMotor.set(deployPower(Settings.IntakeSettings.deployPosition + intakeFudgeFactor));
            // spinIntakeBackward();

            // intake wheels on
            intakeMotorLeftLeader.setControl(m_request.withOutput(1.0));

        } else if (intakeState == IntakeStates.stowed) {
            // stow intake
            intakeDeployMotor.set(deployPower(Settings.IntakeSettings.stowPosition));

            // intake wheels off
            intakeMotorLeftLeader.setControl(m_request.withOutput(0));

        } else if (intakeState == IntakeStates.outtaking) {
            // deploy intake
            intakeDeployMotor.set(deployPower(Settings.IntakeSettings.deployPosition));

            // intake wheels on
            intakeMotorLeftLeader.setControl(m_request.withOutput(-1.0));

        } else if (intakeState == IntakeStates.stationaryDeployed) {
            // deploy intake
            intakeDeployMotor.set(deployPower(Settings.IntakeSettings.deployPosition + intakeFudgeFactor));
            spinIntakeBackward();

            // intake wheels off
            intakeMotorLeftLeader.setControl(m_request.withOutput(0));

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

    public void spinIntakeBackward() {
        if (intakeDeployMotor.getPosition().getValueAsDouble() < 0) {
            intakeMotorLeftLeader.setControl(m_request.withOutput(-1.0));
        }
    }

    public void jiggleIntakeWhileShooting() {
        double maxPosition = -20;
        double minPosition = 0;

        if (Robot.kicker.kickerState == KickerStates.shooting && Robot.hopper.hopperState == HopperStates.indexing) {

        }
    }
}
