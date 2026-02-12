package frc.robot.Logic;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Settings;

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
                SmartDashboard.putNumber("intakeDeployPosition",
                                Robot.intake.intakeDeployMotor.getPosition().getValueAsDouble());
                SmartDashboard.putNumber("intakerightvelocity",
                                Robot.intake.intakeMotorRight.getVelocity().getValueAsDouble());
                SmartDashboard.putNumber("intakeleftvelocity",
                                Robot.intake.intakeMotorLeft.getVelocity().getValueAsDouble());
                // shooter
                SmartDashboard.putNumber("shooterMotorLeftVelocity",
                                Robot.shooter.shooterMotorLeft.getVelocity().getValueAsDouble());
                SmartDashboard.putNumber("shooterMotorRightVelocity",
                                Robot.shooter.shooterMotorRight.getVelocity().getValueAsDouble());
                SmartDashboard.putNumber("shooterHoodMotorPosition",
                                Robot.shooter.shooterHoodMotor.getEncoder().getPosition());
                SmartDashboard.putNumber("IntakeMotorCurrentLeft",
                                Robot.intake.intakeMotorLeft.getSupplyCurrent().getValueAsDouble());
                SmartDashboard.putNumber("IntakeMotorCurrentRight",
                                Robot.intake.intakeMotorRight.getSupplyCurrent().getValueAsDouble());
                // kicker
                SmartDashboard.putNumber("kickerMotorVelocity", Robot.kicker.kickerMotor.getEncoder().getVelocity());

                // Fudge factors
                SmartDashboard.putNumber("shooterAngleFudgeFactorValue", Robot.shooter.angleFudgeFactor);
                SmartDashboard.putNumber("drivebaseAimFudgeFactorValue", Robot.drivebase.aimFudgeFactor);

                // Prompts user input for PID values
                SmartDashboard.putNumber("input shooter kP", Robot.shooter.slot0Configs.kP);
                SmartDashboard.putNumber("input shooter kI", Robot.shooter.slot0Configs.kI);
                SmartDashboard.putNumber("input shooter kD", Robot.shooter.slot0Configs.kD);

                SmartDashboard.putNumber("input intake kP", Robot.intake.intakeMotorController.getP());
                SmartDashboard.putNumber("input intake kI", Robot.intake.intakeMotorController.getI());
                SmartDashboard.putNumber("input intake kD", Robot.intake.intakeMotorController.getD());

                SmartDashboard.putNumber("input kicker kP", Robot.kicker.kickerMotorController.getP());
                SmartDashboard.putNumber("input kicker kI", Robot.kicker.kickerMotorController.getI());
                SmartDashboard.putNumber("input kicker kD", Robot.kicker.kickerMotorController.getD());
        }

        public void getPIDValues() {
                // get pid values from dashboard if the setting is enabled
                if (Settings.DrivebaseSettings.getPIDValuesFromDashboard) {

                        // set shooter pid values
                        Robot.shooter.slot0Configs.kP = SmartDashboard.getNumber("input shooter kP", 0);
                        Robot.shooter.slot0Configs.kI = SmartDashboard.getNumber("input shooter kI", 0);
                        Robot.shooter.slot0Configs.kD = SmartDashboard.getNumber("input shooter kD", 0);

                        // set intake pid values
                        Robot.intake.intakeMotorController.setP(SmartDashboard.getNumber("input intake kP", 0));
                        Robot.intake.intakeMotorController.setI(SmartDashboard.getNumber("input intake kI", 0));
                        Robot.intake.intakeMotorController.setD(SmartDashboard.getNumber("input intake kD", 0));

                        // set kicker pid values
                        Robot.kicker.kickerMotorController.setP(SmartDashboard.getNumber("input kicker kP", 0));
                        Robot.kicker.kickerMotorController.setI(SmartDashboard.getNumber("input kicker kI", 0));
                        Robot.kicker.kickerMotorController.setD(SmartDashboard.getNumber("input kicker kD", 0));
                }

        }
}
