package frc.robot.Subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Logic.Enums.HopperStates;
import frc.robot.Logic.Enums.KickerStates;

public class Hopper {

    public HopperStates hopperState = HopperStates.stationary;
    public static TalonFX indexerMotorTop = new TalonFX(23);
    public static TalonFX indexerMotorBottom = new TalonFX(24);

    public Hopper() {

    }

    public void setMotorPower() {
        if (hopperState == HopperStates.indexing) {
            // set motor to shoot
            indexerMotorTop.set(1);
            indexerMotorBottom.set(-1);
        } else if (hopperState == HopperStates.stationary) {
            // set motor to stationary
            indexerMotorTop.set(0);
            indexerMotorBottom.set(0);
        } else if (hopperState == HopperStates.unjam) {
            // reverse the motor
            indexerMotorTop.set(-1);
            indexerMotorBottom.set(1);
        }
    }
}
