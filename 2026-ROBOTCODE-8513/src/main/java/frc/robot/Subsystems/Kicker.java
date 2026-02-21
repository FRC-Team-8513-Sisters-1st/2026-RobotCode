package frc.robot.Subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Logic.Enums.KickerStates;

public class Kicker {

    public KickerStates kickerState = KickerStates.stationary;
    public SparkMax kickerMotor = new SparkMax(16, MotorType.kBrushless);

    public PIDController kickerMotorController = new PIDController(0.0001, 0, 0);

    public Kicker() {
        // brake mode
    }

    // set motor power based on state
    public void setMotorPower() {
        if (kickerState == KickerStates.shooting) {
            // set motor to shoot
            kickerMotor.set(getMotorPower());
        } else if (kickerState == KickerStates.stationary) {
            // set motor to stationary
            kickerMotor.set(0);
        }
    }

    public double getMotorPower() {
        double currentVelocity = kickerMotor.getEncoder().getVelocity();
        double targetVelocity = 4000;
        double outputPower = kickerMotorController.calculate(currentVelocity, targetVelocity);
        return outputPower + targetVelocity/6000;

    }
}
