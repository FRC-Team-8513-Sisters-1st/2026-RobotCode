package frc.robot.Subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Logic.Enums.HopperStates;

public class Hopper {

    public HopperStates hopperState = HopperStates.stationary;
    public TalonFX indexerMotorTop = new TalonFX(23);
    public TalonFX indexerMotorBottom = new TalonFX(24);

    public double timeIndexerStarts = 0;
    public boolean brokenIndexerTHold = false;
    public int brokenTHoldCounter = 0;

    //should we use a pid controller for indexer?
    public PIDController indexerMotorController = new PIDController(0.1, 0, 0);


    public Hopper() {

    }

    public void setMotorPower() {
        if (hopperState == HopperStates.indexing) {

            SmartDashboard.putNumber("broken THold Counter", brokenTHoldCounter);
            SmartDashboard.putBoolean("broken THold ", brokenIndexerTHold);

            // set motor to shoot
            if (indexerMotorTop.getVelocity().getValueAsDouble() > 30) {
                brokenTHoldCounter++;
            } else {
                brokenTHoldCounter = 0;
            }

            if (brokenTHoldCounter > 10) {
                brokenIndexerTHold = true;
            }

            if (Timer.getFPGATimestamp() - timeIndexerStarts > 0.5 && !brokenIndexerTHold) {
                if (Timer.getFPGATimestamp() - timeIndexerStarts > 1) {
                    brokenIndexerTHold = false;
                    brokenTHoldCounter = 0;
                    timeIndexerStarts = Timer.getFPGATimestamp();
                }
                indexerMotorTop.set(-1);
                indexerMotorBottom.set(1);
            } else {
                indexerMotorTop.set(1);
                indexerMotorBottom.set(-1);
            }
            
        } else if (hopperState == HopperStates.stationary) {
            // set motor to stationary
            indexerMotorTop.set(0);
            indexerMotorBottom.set(0);

            timeIndexerStarts = Timer.getFPGATimestamp();
            brokenIndexerTHold = false;
        } else if (hopperState == HopperStates.unjam) {
            // reverse the motor
            indexerMotorTop.set(-1);
            indexerMotorBottom.set(1);

            timeIndexerStarts = Timer.getFPGATimestamp();
            brokenIndexerTHold = false;
        }
    }
}
