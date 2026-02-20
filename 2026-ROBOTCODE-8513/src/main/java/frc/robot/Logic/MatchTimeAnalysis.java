package frc.robot.Logic;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class MatchTimeAnalysis {
    // returns true when the hub is active
    public boolean isHubActive() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        // If we have no alliance, we cannot be enabled, therefore no hub.
        if (alliance.isEmpty()) {
            return false;
        }
        // Hub is always enabled in autonomous.
        if (DriverStation.isAutonomousEnabled()) {
            return true;
        }
        // At this point, if we're not teleop enabled, there is no hub.
        if (!DriverStation.isTeleopEnabled()) {
            return false;
        }

        // We're teleop enabled, compute.
        double matchTime = DriverStation.getMatchTime();
        String gameData = DriverStation.getGameSpecificMessage();
        // If we have no game data, we cannot compute, assume hub is active, as its
        // likely early in teleop.
        if (gameData.isEmpty()) {
            return true;
        }
        boolean redInactiveFirst = false;
        switch (gameData.charAt(0)) {
            case 'R' -> redInactiveFirst = true;
            case 'B' -> redInactiveFirst = false;
            default -> {
                // If we have invalid game data, assume hub is active.
                return true;
            }
        }

        // Shift was is active for blue if red won auto, or red if blue won auto.
        boolean shift1Active = switch (alliance.get()) {
            case Red -> !redInactiveFirst;
            case Blue -> redInactiveFirst;
        };

        if (matchTime > 130) {
            // Transition shift, hub is active.
            return true;
        } else if (matchTime > 105) {
            // Shift 1
            return shift1Active;
        } else if (matchTime > 80) {
            // Shift 2
            return !shift1Active;
        } else if (matchTime > 55) {
            // Shift 3
            return shift1Active;
        } else if (matchTime > 30) {
            // Shift 4
            return !shift1Active;
        } else {
            // End game, hub always active.
            return true;
        }
    }

    public double getTimeLeftInPeriod() {
        double elapsedMatchTime = Timer.getFPGATimestamp() - Robot.teleopInit.teleStartTime;
        if (elapsedMatchTime > 0 && elapsedMatchTime <= 10) {
            double timeLeftInPeriod = 10 - elapsedMatchTime;
            return timeLeftInPeriod;
        } else if (elapsedMatchTime > 10 && elapsedMatchTime <= 35) {
            double timeLeftInPeriod = 35 - elapsedMatchTime;
            return timeLeftInPeriod;
        } else if (elapsedMatchTime > 35 && elapsedMatchTime <= 60) {
            double timeLeftInPeriod = 60 - elapsedMatchTime;
            return timeLeftInPeriod;
        } else if (elapsedMatchTime > 60 && elapsedMatchTime <= 85) {
            double timeLeftInPeriod = 85 - elapsedMatchTime;
            return timeLeftInPeriod;
        } else if (elapsedMatchTime > 85 && elapsedMatchTime <= 110) {
            double timeLeftInPeriod = 110 - elapsedMatchTime;
            return timeLeftInPeriod;
        } else if (elapsedMatchTime > 110 && elapsedMatchTime <= 140) {
            double timeLeftInPeriod = 140 - elapsedMatchTime;
            return timeLeftInPeriod;
        }
        
    }
}
