package frc.robot.Logic;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class Dashboard {
    public Field2d trajField2d = new Field2d();

    public void updateDashboard() {

        SmartDashboard.putData("trajGoalPose", trajField2d);
        SmartDashboard.putString("intakeState", Robot.intake.intakeState.name());
        SmartDashboard.putNumber("intakeDeployPosition", Robot.intake.intakeDeployMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("actualHeading", Robot.drivebase.yagslDrive.getOdometryHeading().getDegrees());
        SmartDashboard.putNumber("goalHeading", Robot.drivebase.goalHeading.getDegrees());


    }
}
