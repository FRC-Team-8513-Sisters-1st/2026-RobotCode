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
        double elapsedMatchTime = Timer.getFPGATimestamp() - Robot.teleop.teleStartTime;
        if (elapsedMatchTime >= 0 && elapsedMatchTime < 20) {
            double timeLeftInPeriod = 20 - elapsedMatchTime;
            return timeLeftInPeriod;
        } else if (elapsedMatchTime >= 20 && elapsedMatchTime < 30) {
            double timeLeftInPeriod = 30 - elapsedMatchTime;
            return timeLeftInPeriod;
        } else if (elapsedMatchTime >= 30 && elapsedMatchTime < 55) {
            double timeLeftInPeriod = 55 - elapsedMatchTime;
            return timeLeftInPeriod;
        } else if (elapsedMatchTime >= 55 && elapsedMatchTime < 80) {
            double timeLeftInPeriod = 80 - elapsedMatchTime;
            return timeLeftInPeriod;
        } else if (elapsedMatchTime >= 80 && elapsedMatchTime < 105) {
            double timeLeftInPeriod = 105 - elapsedMatchTime;
            return timeLeftInPeriod;
        } else if (elapsedMatchTime >= 105 && elapsedMatchTime < 130) {
            double timeLeftInPeriod = 130 - elapsedMatchTime;
            return timeLeftInPeriod;
        } else if (elapsedMatchTime >= 130 && elapsedMatchTime < 160) {
            double timeLeftInPeriod = 160 - elapsedMatchTime;
            return timeLeftInPeriod;
        } else {
            return 0;
        }
    }

    public String getShift() {
        double elapsedMatchTime = Timer.getFPGATimestamp() - Robot.teleop.teleStartTime;
        if (elapsedMatchTime >= 0 && elapsedMatchTime < 20) {
            return "Auto";
        } else if (elapsedMatchTime >= 20 && elapsedMatchTime < 30) {
            return "Transition";
        } else if (elapsedMatchTime >= 30 && elapsedMatchTime < 55) {
            return "Shift 1";
        } else if (elapsedMatchTime >= 55 && elapsedMatchTime < 80) {
            return "Shift 2";
        } else if (elapsedMatchTime >= 80 && elapsedMatchTime < 105) {
            return "Shift 3";
        } else if (elapsedMatchTime >= 105 && elapsedMatchTime < 130) {
            return "Shift 4";
        } else {
            return "End Game";
        }
    }

    public String activeOrInactive() {
        if (isHubActive()){
            return "Active";
        } else {
            return "Inactive";
        }
    }
}
