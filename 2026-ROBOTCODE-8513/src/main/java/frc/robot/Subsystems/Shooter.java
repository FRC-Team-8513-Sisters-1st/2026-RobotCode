package frc.robot.Subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Logic.Settings;

public class Shooter {

    public boolean isShooting = false;
    public boolean readyToShoot = false;

    public PIDController spinUpShooterPID = new PIDController(Settings.ShooterSettings.SpinUpShooterPIDFConstants.kP,
                                                                Settings.ShooterSettings.SpinUpShooterPIDFConstants.kI,
                                                                Settings.ShooterSettings.SpinUpShooterPIDFConstants.kD);

    public PIDController maintainShooterPID = new PIDController(Settings.ShooterSettings.MaintainShooterPIDFConstants.kP,
                                                                Settings.ShooterSettings.MaintainShooterPIDFConstants.kI,
                                                                Settings.ShooterSettings.MaintainShooterPIDFConstants.kD);

    public TalonFX shooterMotor1 = new TalonFX(Settings.ShooterSettings.shooterMotor1CANID);
    public TalonFX shooterMotor2 = new TalonFX(Settings.ShooterSettings.shooterMotor2CANID);


    public Shooter(){
        //redo this with magic motion 
    }

    
}
