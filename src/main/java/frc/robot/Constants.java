package frc.robot;

public class Constants {

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


        public static final int cfrontLeftDriveMotorID = 1;
        public static final int cfrontRightDriveMotorID = 2;
        public static final int cbackLeftDriveMotorID = 3;
        public static final int cbackRightDriveMotorID = 4;

        public static final int bFrontLeftID = 4;
		public static final int bbackLeftID = 5;
		public static final int bfrontRightID = 6;
		public static final int bbackRightID = 7;

		public static final double autoDrivePercent = 0.6;
    }

        public static final int pigeonID = 20;
}   

