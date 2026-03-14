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
        DoNothing, Depot_OneCycle, Outpost_OneCycle_Close, Outpost_OneCycle_Mid, Outpost_OneCycle_Far, Outpost, Depot, 
        OliviaAttemptGoOverBump, 
        Depot_FullAcross_OneCycle, Outpost_FullAcross_OneCycle, 
        Outpost_AnyTwoParts, Depot_AnyTwoParts, Outpost_OneCycle_MidNORETURN, Outpost_OneCycle_CloseNORETURN,
        Outpost_OneCycle_FarNORETURN
    }

    public enum TCPChooser {
        autoDetectWinnerOfAuto, blueWonAuto, redWonAuto
    }

}
