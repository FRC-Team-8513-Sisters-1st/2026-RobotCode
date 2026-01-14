package frc.robot.Logic;

public class Settings {

    public static class DrivebaseSettings{
        public static final double maxVelocityMPS = 3.0;
        public static final double maxRotationalVelocityRPS = 6.28;

        public static class RotationPIDConstants{
            public static final double kP = 0.1;
            public static final double kI = 0;
            public static final double kD = 0;

        }
    }

    public static class TeleopSettings{

        public static class DriverJoystick{
            public static final int port = 0;
            public static final int xAxis = 1;
            public static final int yAxis = 0;
            public static final int rAxis = 4;
            
            public static final double deadband = 0.1;

            public static final double rotationalJoystickSensitivity = 1;

        }
        
    }
    
}
