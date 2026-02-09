package frc.robot.Logic;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class Dashboard {
    public Field2d trajField2d = new Field2d();

    public void updateDashboard() {

        // Robot position
        SmartDashboard.putData("trajGoalPose", trajField2d);
        SmartDashboard.putNumber("actualHeading", Robot.drivebase.yagslDrive.getOdometryHeading().getDegrees());
        SmartDashboard.putNumber("goalHeading", Robot.drivebase.goalHeading.getDegrees());

        // Auto selection
        SmartDashboard.putString("Auto selected", Robot.auto.autoRoutine.name());

        // Subsystem states
        SmartDashboard.putString("intakeState", Robot.intake.intakeState.name());
        SmartDashboard.putString("shooterStates", Robot.shooter.shooterState.name());
        SmartDashboard.putString("kickerStates", Robot.kicker.kickerState.name());
        SmartDashboard.putString("hopperStates", Robot.hopper.hopperState.name());

        // Motor locations/positions
        // intake
        SmartDashboard.putNumber("intakeDeployPosition", Robot.intake.intakeDeployMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("intakerightvelocity", Robot.intake.intakeMotorRight.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("intakeleftvelocity", Robot.intake.intakeMotorLeft.getVelocity().getValueAsDouble());
        // shooter
        SmartDashboard.putNumber("shooterMotorLeft", Robot.shooter.shooterMotorLeft.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("shooterMotorRight", Robot.shooter.shooterMotorRight.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("shooterHoodMotor", Robot.shooter.shooterHoodMotor.getEncoder().getPosition());
        // kicker
        SmartDashboard.putNumber("kickerMotor", Robot.kicker.kickerMotor.getEncoder().getVelocity());
        // hopper
                

        // Fudge factors
        SmartDashboard.putNumber("shooterAngleFudgeFactorValue", Robot.shooter.angleFudgeFactor);
        SmartDashboard.putNumber("drivebaseAimFudgeFactorValue", Robot.drivebase.aimFudgeFactor);


    }
}
