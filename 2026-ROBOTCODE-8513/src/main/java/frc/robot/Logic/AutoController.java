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

}

//     public void initAuto() {
//         updateAutoRoutineFromDashboard();
//         autoStep = 0;
//         timeStepStarted = Timer.getFPGATimestamp();
//     }

// //     public void autoPeriodic() {

       
// // }
