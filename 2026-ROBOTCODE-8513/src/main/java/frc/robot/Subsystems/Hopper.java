package frc.robot.Subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Logic.Enums.HopperStates;

public class Hopper {

    public HopperStates hopperState = HopperStates.stationary;
    public static TalonFX indexerMotorTop = new TalonFX(23);
    public static TalonFX indexerMotorBottom = new TalonFX(24);

    public Hopper() {

    }
}
