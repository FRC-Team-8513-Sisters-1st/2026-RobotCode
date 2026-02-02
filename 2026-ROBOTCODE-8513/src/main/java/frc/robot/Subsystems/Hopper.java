package frc.robot.Subsystems;

import com.revrobotics.spark.SparkMax;

import frc.robot.Robot;

import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Hopper {
    public static SparkMax hopperMotorLeft = new SparkMax(20, MotorType.kBrushless);
    public static SparkMax hopperMotorRight = new SparkMax(21, MotorType.kBrushless);
    public boolean motorOn = false;

    public Hopper() {
    }

    public void setMotorPower() {
        if (Robot.teleop.driverXboxController.getRawButton(6)) {
            motorOn = true;
            hopperMotorLeft.set(1);
            hopperMotorRight.set(1);

        } else if (Robot.teleop.driverXboxController.getRawButton(6) && motorOn) {
            motorOn = false;
            hopperMotorLeft.set(0);
            hopperMotorRight.set(0);
        }
    }

}
