package frc.robot.Logic;

public class Enums {
    public enum IntakeStates {
        intaking, outtaking, stowed, stationaryDeployed
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
        DoNothing, TestAuto, Depot_OneCycle, Outpost_OneCycle
    }

}
