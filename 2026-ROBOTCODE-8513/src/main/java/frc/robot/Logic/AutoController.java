package frc.robot.Logic;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Logic.Enums.AutoRoutines;
import frc.robot.Logic.Enums.HopperStates;
import frc.robot.Logic.Enums.IntakeStates;
import frc.robot.Logic.Enums.KickerStates;
import frc.robot.Logic.Enums.ShooterStates;

public class AutoController {

    public SendableChooser<String> autoSelector;
    public AutoRoutines autoRoutine = AutoRoutines.DoNothing;

    public int autoStep;
    double timeStepStarted = 0;

    public AutoController() {
        // create auto selector with each enum option
        autoSelector = new SendableChooser<>();
        autoSelector.setDefaultOption(AutoRoutines.values()[0].toString(), AutoRoutines.values()[0].toString());
        for (int i = 1; i < AutoRoutines.values().length; i++) {
            if (AutoRoutines.values()[i].toString().charAt(0) != '~') {
                autoSelector.addOption(AutoRoutines.values()[i].toString(), AutoRoutines.values()[i].toString());
            }

        }
        SmartDashboard.putData("Auton Selector", autoSelector);
    }

    public void initAuto() {
        updateAutoRoutineFromDashboard();
        autoStep = 0;
        timeStepStarted = Timer.getFPGATimestamp();
    }

    public void autoPeriodic() {

        switch (autoRoutine) {
            case DoNothing:
                Robot.drivebase.yagslDrive.lockPose();
                break;

            case TestAuto:
                switch (autoStep) {
                    case 0:
                        Robot.shooter.shooterState = ShooterStates.stationary;
                        Robot.hopper.hopperState = HopperStates.stationary;
                        Robot.kicker.kickerState = KickerStates.stationary;
                        Robot.intake.intakeState = IntakeStates.stowed;

                        Robot.drivebase.initPath("TestAuto-step5");

                        // path is over
                        timeStepStarted = Timer.getFPGATimestamp();
                        autoStep = 10;
                        break;

                    case 10:
                        Robot.drivebase.followLoadedPath();
                        Robot.shooter.shooterState = ShooterStates.shooting;
                        Robot.hopper.hopperState = HopperStates.indexing;
                        Robot.kicker.kickerState = KickerStates.shooting;
                        Robot.intake.intakeState = IntakeStates.stowed;

                        if (Timer.getFPGATimestamp() - timeStepStarted > 3) {
                            timeStepStarted = Timer.getFPGATimestamp();
                            autoStep = 15;
                        }
                        break;
                    
                    case 15:
                        Robot.shooter.shooterState = ShooterStates.stationary;
                        Robot.hopper.hopperState = HopperStates.stationary;
                        Robot.kicker.kickerState = KickerStates.stationary;
                        Robot.intake.intakeState = IntakeStates.stowed;

                        Robot.drivebase.initPath("TestAuto-step15");

                        // path is over
                        timeStepStarted = Timer.getFPGATimestamp();
                        autoStep = 20;
                        break;
                    case 20:
                        Robot.drivebase.followLoadedPath();

                }
                break;
        }
    }

    public void updateAutoRoutineFromDashboard() {
        try {
            autoRoutine = AutoRoutines.valueOf(autoSelector.getSelected());
        } catch (Exception e) {
            autoRoutine = AutoRoutines.DoNothing;
        }
    }

}
