package frc.robot.Logic;

public class Enums {
    public enum IntakeStates {
        intaking, outtaking, stowed, stationaryDeployed, shooting
    }
    public enum ClimberStates {
        stowed, deployed, climbed
    }
    public enum HopperStates {
        stationary, indexing, unjam
    }
    public enum KickerStates {
        stationary, intaking, shooting
    }
    public enum ShooterStates {
        stationary, shooting
    }
    public enum DrivebaseStates {
        locked, drive 
    }

    public enum AutoRoutines {
        DoNothing, Depot_OneCycle, Outpost_OneCycle, Outpost, Depot, MoveForward1Meter, MoveForward1MeterComeBack, RotateWhileDriving, DriveAtAnAngle, OliviaAttemptGoOverBump, Depot_FullAcross_OneCycle, Outpost_FullAcross_OneCycle, Depot_Depot_Outpost, Outpost_TwoCycle, Depot_TwoCycle
    }

    public enum TCPChooser {
        autoDetectWinnerOfAuto, blueWonAuto, redWonAuto
    }

}
