package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Settings;
import frc.robot.Logic.Enums.ShooterStates;

public class Shooter {

    public static TalonFX shooterMotorLeft = new TalonFX(13);
    public static TalonFX shooterMotorRight = new TalonFX(14);
    public static SparkMax shooterHoodMotor = new SparkMax(15, MotorType.kBrushless);

    public PIDController shooterMotorController = new PIDController(0.1, 0, 0);

    public ShooterStates shooterState = ShooterStates.stationary;

    public boolean useInternalController = true;

    // in init function, set slot 0 gains
    Slot0Configs slot0Configs = new Slot0Configs();

    // create a velocity closed-loop request, voltage output, slot 0 configs
    public final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

    public Shooter() {
        slot0Configs.kV = 0; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kP = 0.5; // An error of 1 rps results in 0.11 V output
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

        } else if (shooterState == ShooterStates.stationary && useInternalController == false) {
            shooterMotorLeft.set(0);
            shooterMotorRight.set(0);
        }

        // shooter controller if using interal pid
        double targetV = 41;
        SmartDashboard.putNumber("ShooterV", shooterMotorRight.getVelocity().getValueAsDouble());
        if (shooterState == ShooterStates.shooting && useInternalController == true) {
            shooterMotorLeft.setControl(m_request.withVelocity(-targetV).withFeedForward(-RPStoVoltage(targetV)));
            shooterMotorRight.setControl(m_request.withVelocity(targetV).withFeedForward(RPStoVoltage(targetV)));

        } else if (shooterState == ShooterStates.stationary && useInternalController == true) {
            shooterMotorLeft.set(0);
            shooterMotorRight.set(0);

        }

        setHoodAngle();
    }

    double targetVelocity;

    public double updateMotorPower() {
        double currentVelocity = shooterMotorLeft.getVelocity().getValueAsDouble();
        targetVelocity = 3000;
        double outputPower = shooterMotorController.calculate(currentVelocity, targetVelocity);
        return outputPower + targetVelocity / 6140;
    }

    public void setHoodAngle() {

        shooterHoodMotor.set(hoodAnglePower(hoodCalcAngleToPower()));
    }

    public double hoodAnglePower(double targetPosition) {
        double currentPosition = shooterHoodMotor.getEncoder().getPosition();
        double outputPower = shooterMotorController.calculate(currentPosition, targetPosition);
        return outputPower;
    }

    public double hoodCalcAngleToPower() {
        // find theta
        double theta = Math.atan((Math.pow(targetVelocity, 2) + Math
                .sqrt(Math.pow(targetVelocity, 4) - (9.8) * (9.8
                        * Math.pow(Robot.drivebase.getDistanceBetweenTwoPoses(Robot.drivebase.yagslDrive.getPose(),
                                Settings.FieldInfo.redHubCenterPoint), 2)
                        + 2 * (Settings.FieldInfo.hubHeight - Settings.PhysicalRobotValues.robotHeight)
                                * (Math.pow(targetVelocity, 2))))
                / (9.8 * Robot.drivebase.getDistanceBetweenTwoPoses(Robot.drivebase.yagslDrive.getPose(),
                        Settings.FieldInfo.redHubCenterPoint))));
        // find connection btwn theta and encoder position
        double updatedPosition = 30;
        return updatedPosition;
    }

    public double RPStoVoltage(double RPS) {
        double voltage = (RPS + 0.773138) / 8.70426;
        return voltage;
    }
}
