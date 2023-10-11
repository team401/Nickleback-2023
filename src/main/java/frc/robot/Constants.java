package frc.robot;

public class Constants {

    public enum DriveMode {
        TANKDRIVE,
        ARCADEDRIVE
    }

    public enum ChassisMode {
        B_TEAM,
        C_TEAM,
    }
    
    public final class ArmConstants {

        public static final int wristMotorID = 19;  //19?
        public static final int leftIntakeMotorID = 29;
        public static final int rightIntakeMotorID = 50;
        public static final int topIntakeMotorID = 3; //?

        public static final double wristkP = 0.1;
        public static final double wristkD = 0.0;

        public static final double intakePosition = -35;
        public static final double upperShootPosition = -3;
        public static final double stowShootPosition = 0;
        public static final double lowShootVoltage = -0.1;
        public static final double midShootVoltage = -0.2;
        public static final double highShootVoltage = -0.5;
        public static final double intakeVoltage = 0.2;
        public static final double spitVoltage = 0.5;

    }

    public static final class DriveConstants {
        public static final int frontLeftID = 52;
		public static final int backLeftID = 53;
		public static final int frontRightID = 14;
		public static final int backRightID = 16;

		public static final double autoDrivePercent = 0.6;
    }


}
