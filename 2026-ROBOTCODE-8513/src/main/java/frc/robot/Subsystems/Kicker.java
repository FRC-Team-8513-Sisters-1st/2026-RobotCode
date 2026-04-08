package frc.robot.Subsystems;

import com.revrobotics.spark.SparkMax;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Robot;
import frc.robot.Logic.Enums.KickerStates;

public class Kicker {

    public KickerStates kickerState = KickerStates.stationary;
    public TalonFX kickerMotor = new TalonFX(17);

    public PIDController kickerMotorController = new PIDController(0.0001, 0, 0);

    public Slot0Configs slot0Configs = new Slot0Configs();

    public final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

    private double targetV;


    public Kicker() {
        // brake mode
        slot0Configs.kV = 0; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kP = 0.55; // An error of 1 rps results in 0.11 V output
        slot0Configs.kI = 0.05; // no output for integrated error
        slot0Configs.kD = 0; // no output for error derivative
    }

    // set motor power based on state
    public void setMotorPower() {

        targetV = 100;
        
        if (kickerState == KickerStates.shooting) {
            // set motor to shoot
            kickerMotor.setControl(m_request.withVelocity(targetV).withFeedForward(Robot.shooter.RPStoVoltage(targetV)));
        } else if (kickerState == KickerStates.stationary) {
            // set motor to stationary
            kickerMotor.set(0);
        } else if (kickerState == KickerStates.intaking) {
            // set motor to reverse
            kickerMotor.set(-1);
        }
    }
}
