package frc.robot.Subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Settings;
import frc.robot.Logic.Enums.IntakeStates;

public class Intake {
    public IntakeStates intakeState = IntakeStates.stowed; 

    public static TalonFX intakeMotorLeft = new TalonFX(31);
    public static TalonFX intakeMotorRight = new TalonFX(32);
    public static TalonFX intakeDeployMotor = new TalonFX(33);

    public PIDController intakeMotorController = new PIDController(0.0001, 0.000001, 0);

    public Intake(){

    }
    
    public void setMotorPower() {

        if (intakeState == IntakeStates.intaking) {
            // deploy intake
            intakeDeployMotor.set(deployPower(Settings.IntakeSettings.deployPosition));
            
            // intake wheels on
            intakeMotorLeft.set(1);
            intakeMotorRight.set(1);

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
            intakeMotorRight.set(-1);
        }

    }

    public double deployPower(double targetPosition) {
        double currentPosition = intakeDeployMotor.getPosition().getValueAsDouble();
        double outputPower = intakeMotorController.calculate(currentPosition, targetPosition);
        return outputPower;
    }
}
