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
                                Robot.intake.intakeMotorRightFollower.getVelocity().getValueAsDouble());
                SmartDashboard.putNumber("intakeleftvelocity",
                                Robot.intake.intakeMotorLeftLeader.getVelocity().getValueAsDouble());
                SmartDashboard.putNumber("left intake motor power", Robot.intake.intakeMotorLeftLeader.get());
                SmartDashboard.putNumber("right intake motor power", Robot.intake.intakeMotorRightFollower.get());
                SmartDashboard.putNumber("IntakeMotorCurrentLeft",
                                Robot.intake.intakeMotorLeftLeader.getSupplyCurrent().getValueAsDouble());
                SmartDashboard.putNumber("IntakeMotorCurrentRight",
                                Robot.intake.intakeMotorRightFollower.getSupplyCurrent().getValueAsDouble());

                // shooter
                SmartDashboard.putNumber("shooterMotorLeftVelocity",
                                Robot.shooter.shooterMotorLeftLeader.getVelocity().getValueAsDouble());
                SmartDashboard.putNumber("shooterMotorLeftCurrent",
                                Robot.shooter.shooterMotorLeftLeader.getStatorCurrent().getValueAsDouble());
                SmartDashboard.putNumber("shooterMotorRightVelocity",
                                Robot.shooter.shooterMotorRightFollower.getVelocity().getValueAsDouble());
                SmartDashboard.putNumber("shooterHoodMotorPosition",
                                Robot.shooter.shooterHoodMotor.getEncoder().getPosition());
                SmartDashboard.putNumber("ShooterMotorCurrentLeft",
                                Robot.shooter.shooterMotorLeftLeader.getSupplyCurrent().getValueAsDouble());
                SmartDashboard.putNumber("ShooterMotorCurrentRight",
                                Robot.shooter.shooterMotorRightFollower.getSupplyCurrent().getValueAsDouble());
                // kicker
                SmartDashboard.putNumber("kickerMotorVelocity", Robot.kicker.kickerMotor.getEncoder().getVelocity());
                SmartDashboard.putNumber("kickerMotorCurrent",  Robot.kicker.kickerMotor.getOutputCurrent());

                // hopper
                SmartDashboard.putNumber("hopperMotorVelocityTop", Robot.hopper.indexerMotorTop.getVelocity().getValueAsDouble());
                SmartDashboard.putNumber("hopperMotorVelocityBottom", Robot.hopper.indexerMotorBottom.getVelocity().getValueAsDouble());
                SmartDashboard.putNumber("IndexerMotorCurrentLeft",
                                Robot.hopper.indexerMotorBottom.getSupplyCurrent().getValueAsDouble());
                SmartDashboard.putNumber("IndexerMotorCurrentRight",
                                Robot.hopper.indexerMotorTop.getSupplyCurrent().getValueAsDouble());


                // Fudge factors
                SmartDashboard.putNumber("shooterAngleFudgeFactorValue", Robot.shooter.angleFudgeFactor);
                SmartDashboard.putNumber("drivebaseAimFudgeFactorValue", Robot.drivebase.aimFudgeFactor);

                // Prompts user input for PID values
                SmartDashboard.putNumber("output shooter kP", Robot.shooter.slot0Configs.kP);
                SmartDashboard.putNumber("output shooter kI", Robot.shooter.slot0Configs.kI);
                SmartDashboard.putNumber("output shooter kD", Robot.shooter.slot0Configs.kD);

                SmartDashboard.putNumber("output intake kP", Robot.intake.intakeMotorController.getP());
                SmartDashboard.putNumber("output intake kI", Robot.intake.intakeMotorController.getI());
                SmartDashboard.putNumber("output intake kD", Robot.intake.intakeMotorController.getD());

                SmartDashboard.putNumber("output kicker kP", Robot.kicker.kickerMotorController.getP());
                SmartDashboard.putNumber("output kicker kI", Robot.kicker.kickerMotorController.getI());
                SmartDashboard.putNumber("output kicker kD", Robot.kicker.kickerMotorController.getD());

                SmartDashboard.putNumber("output indexer kP", Robot.hopper.indexerMotorController.getP());
                SmartDashboard.putNumber("output indexer kI", Robot.hopper.indexerMotorController.getI());
                SmartDashboard.putNumber("output indexer kD", Robot.hopper.indexerMotorController.getD());
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

                        // set indexer pid values
                        Robot.hopper.indexerMotorController.setP(SmartDashboard.getNumber("input indexer kP", 0));
                        Robot.hopper.indexerMotorController.setI(SmartDashboard.getNumber("input indexer kI", 0));
                        Robot.hopper.indexerMotorController.setD(SmartDashboard.getNumber("input indexer kD", 0));

                }

        }
}
