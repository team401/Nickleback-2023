package frc.robot;

public class Constants {
    
    public final class ArmConstants {

        public static final int wristMotorID = 19;  //19?
        public static final int leftIntakeMotorID = 29;
        public static final int rightIntakeMotorID = 50;

    }

    public class CANDevices{

        public static final int frontLeftDriveMotorID = 52; // was2;
        public static final int frontLeftRotationMotorID = 1;

        public static final int frontRightDriveMotorID = 14; //4;
        public static final int frontRightRotationMotorID = 3;

        public static final int backLeftDriveMotorID = 53;  //was 8
        public static final int backLeftRotationMotorID = 7;

        public static final int backRightDriveMotorID = 16; //was 6
        public static final int backRightRotationMotorID = 5;

        public static final int frontLeftRotationEncoderID = 9;
        public static final int frontRightRotationEncoderID = 10;
        public static final int backLeftRotationEncoderID = 12;
        public static final int backRightRotationEncoderID = 11;
    }

    public static final class DriveConstants {
        public static final int frontLeftID = 52;
		public static final int backLeftID = 53;
		public static final int frontRightID = 14;
		public static final int backRightID = 16;

		public static final double autoDrivePercent = 0.6;
    }


}
