package frc.robot;

public class Constants {
    
    public final class ArmConstants {

        public static final int wristMotorID = 0;
        public static final int leftMotorID = 1;
        public static final int rightMotorID = 2;
        public static final int intakeMotorID = 3;

    }

    public class CANDevices{

        public static final int frontLeftDriveMotorID = 2;
        public static final int frontLeftRotationMotorID = 1;

        public static final int frontRightDriveMotorID = 4;
        public static final int frontRightRotationMotorID = 3;

        public static final int backLeftDriveMotorID = 8;
        public static final int backLeftRotationMotorID = 7;

        public static final int backRightDriveMotorID = 6;
        public static final int backRightRotationMotorID = 5;

        public static final int frontLeftRotationEncoderID = 9;
        public static final int frontRightRotationEncoderID = 10;
        public static final int backLeftRotationEncoderID = 12;
        public static final int backRightRotationEncoderID = 11;
    }

    public static final class DriveConstants {

        public static final double[] driveKps = {0.7, 0.4, 0.7, 0.7};
        public static final double[] driveKds = {3.5, 2.5, 3.7, 3.5};

        public static final double[] rotationKps = {7, 7, 7, 7};
        public static final double[] rotationKds = {0, 0, 0, 0};

        public static final double driveSnapKp = 1.5;
        public static final double driveSnapKi = 0;
        public static final double driveSnapKd = 0;

        public static final double maxDriveSpeed = 5;
        public static final double maxTurnRate = 2 * Math.PI;

        public static final double driveJoystickDeadbandPercent = 0.12;
        public static final double driveMaxJerk = 200.0;

        public static final double poseMoveTranslationkP = 1;
        public static final double poseMoveTranslationMaxVel = 3;
        public static final double poseMoveTranslationMaxAccel = 3;

        public static final double poseMoveRotationkP = 0.05;
        public static final double poseMoveRotationMaxVel = Math.PI;
        public static final double poseMoveRotationMaxAccel = Math.PI;


        public static final int frontLeftID = 4;
		public static final int backLeftID = 5;
		public static final int frontRightID = 6;
		public static final int backRightID = 7;

		public static final double autoDrivePercent = 0.6;
    }

}
