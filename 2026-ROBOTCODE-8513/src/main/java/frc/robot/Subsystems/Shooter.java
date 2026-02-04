package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Settings;
import frc.robot.Logic.Enums.ShooterStates;

public class Shooter {

    public static TalonFX shooterMotorLeft = new TalonFX(13);
    public static TalonFX shooterMotorRight = new TalonFX(14);
    public static TalonFX shooterHoodMotor = new TalonFX(15);


    public PIDController shooterMotorController = new PIDController(0.0001, 0.000001, 0);

    public ShooterStates shooterState = ShooterStates.stationary;

    public boolean useInternalController = true;

    // in init function, set slot 0 gains
    Slot0Configs slot0Configs = new Slot0Configs();
    

    // create a velocity closed-loop request, voltage output, slot 0 configs
    public final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

    public Shooter() {
        slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kP = 0.11; // An error of 1 rps results in 0.11 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0; // no output for error derivative
    }

    public void initShooter() {
        shooterMotorLeft.getConfigurator().apply(slot0Configs);
        shooterMotorRight.getConfigurator().apply(slot0Configs);
    }

    public void setMotorPower() {
        // shooter controller if not using interal pid
        if (shooterState == ShooterStates.shooting && useInternalController == false) {
            shooterMotorLeft.set(updateMotorPower());
            shooterMotorRight.set(updateMotorPower());

        } else if (shooterState == ShooterStates.stationary && useInternalController == false){
            shooterMotorLeft.set(0);
            shooterMotorRight.set(0);
        }

        // shooter controller if using interal pid
        if (shooterState == ShooterStates.shooting && useInternalController == true) {
            shooterMotorLeft.setControl(m_request.withVelocity(50).withFeedForward(0.5));
            shooterMotorRight.setControl(m_request.withVelocity(50).withFeedForward(0.5));

        } else if (shooterState == ShooterStates.stationary && useInternalController == true){
            shooterMotorLeft.setControl(m_request.withVelocity(0).withFeedForward(0));
            shooterMotorRight.setControl(m_request.withVelocity(0).withFeedForward(0));

        }

        setHoodAngle();
    }

    public double updateMotorPower() {
        double currentVelocity = shooterMotorLeft.getVelocity().getValueAsDouble();
        double targetVelocity = 3000;
        double outputPower = shooterMotorController.calculate(currentVelocity, targetVelocity);
        return outputPower + targetVelocity/6140;
    }

    public void setHoodAngle() {
        // calculation based on distance
        shooterHoodMotor.set(hoodAnglePower(Settings.ShooterSettings.hoodPosition));
    }

    public double hoodAnglePower(double targetPosition) {
        double currentPosition = shooterHoodMotor.getPosition().getValueAsDouble();
        double outputPower = shooterMotorController.calculate(currentPosition, targetPosition);
        return outputPower;
    }
}
