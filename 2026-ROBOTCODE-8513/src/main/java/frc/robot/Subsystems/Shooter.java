package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Logic.Enums.ShooterStates;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class Shooter {

    public static TalonFX shooterMotorLeft = new TalonFX(13);
    public static TalonFX shooterMotorRight = new TalonFX(14);
    public static SparkMax shooterHoodMotor = new SparkMax(15, MotorType.kBrushless);

    public PIDController shooterMotorController = new PIDController(0.1, 0, 0);

    public ShooterStates shooterState = ShooterStates.stationary;

    public boolean useInternalController = true;

    public static double angleFudgeFactor = 0;

    // in init function, set slot 0 gains
    public Slot0Configs slot0Configs = new Slot0Configs();

    // create a velocity closed-loop request, voltage output, slot 0 configs
    public final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);
    public InterpolatingDoubleTreeMap distToHoodEncoderValuesTable = new InterpolatingDoubleTreeMap();

    public Shooter() {
        // internal pid controller shooter motors
        slot0Configs.kV = 0; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kP = 0.5; // An error of 1 rps results in 0.11 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0; // no output for error derivative

        // interpolating value
        distToHoodEncoderValuesTable.put(0.0, 0.0);
        distToHoodEncoderValuesTable.put(1.0, 10.0);
        distToHoodEncoderValuesTable.put(2.0, 30.0);
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
        double targetV = 47;
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

    // pid controller when not using internal
    double targetVelocity;

    public double updateMotorPower() {
        double currentVelocity = shooterMotorLeft.getVelocity().getValueAsDouble();
        targetVelocity = 3000;
        double outputPower = shooterMotorController.calculate(currentVelocity, targetVelocity);
        return outputPower + targetVelocity / 6140;
    }

    public void setHoodAngle() {
        double distanceBetweenCurrentAndGoalInMeters = Robot.drivebase
                .getDistanceBetweenTwoPoses(Robot.drivebase.yagslDrive.getPose(), Robot.drivebase.goalAimPose);

        shooterHoodMotor
                .set(hoodAnglePower(getInterpolatedEncoderValueDistanceToHood(distanceBetweenCurrentAndGoalInMeters)));
    }

    // input target encoder position from the interpolation chart, PIDs, then
    // returns the power to maintain that position
    public double hoodAnglePower(double targetPosition) {
        double currentPosition = shooterHoodMotor.getEncoder().getPosition();
        double outputPower = shooterMotorController.calculate(currentPosition, targetPosition);
        return outputPower;
    }

    // input distance, returns encoder position
    public double getInterpolatedEncoderValueDistanceToHood(double distanceFromGoal) {
        return distToHoodEncoderValuesTable.get(distanceFromGoal + angleFudgeFactor);
    }

    public double RPStoVoltage(double RPS) {
        double voltage = (RPS + 0.773138) / 8.70426;
        return voltage;
    }

    public boolean readyToShoot() {
        if (Math.abs(Robot.drivebase.yagslDrive.getOdometryHeading().getDegrees() - Robot.drivebase.goalHeading.getDegrees()) > Settings.AutoSettings.Thresholds.drivebaseRotationTHold) {
            return false;
        }
        return true;
    }
}