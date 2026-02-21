package frc.robot.Subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Logic.Enums.HopperStates;

public class Hopper {

    public HopperStates hopperState = HopperStates.stationary;
    public TalonFX indexerMotorTop = new TalonFX(23);
    public TalonFX indexerMotorBottom = new TalonFX(24);

    //should we use a pid controller for indexer?
    public PIDController indexerMotorController = new PIDController(0.1, 0, 0);


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
