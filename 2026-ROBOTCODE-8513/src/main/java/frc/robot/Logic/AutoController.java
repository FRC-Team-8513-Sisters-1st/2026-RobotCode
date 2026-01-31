package frc.robot.Logic;

import frc.robot.Robot;

public class AutoController {
    public void initAuto() {
        Robot.drivebase.initPath("Move Forward");
    }

    public void runAuto() {
        
        if(Robot.drivebase.followLoadedPath()){
            Robot.drivebase.yagslDrive.lockPose();
        }
    }
}
