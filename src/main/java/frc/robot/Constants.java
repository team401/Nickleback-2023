package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

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

        public static final double trackWidth = Units.inchesToMeters(17.75);
        public static final double wheelBase = Units.inchesToMeters(23.75);

        public static final int frontLeftID = 52;
		public static final int backLeftID = 53;
		public static final int frontRightID = 14;
		public static final int backRightID = 16;

        public static final int frontLeftRotationMotorID = 0;
        public static final int frontRightRotationMotorID = 0;
        public static final int backLeftRotationMotorID = 0;
        public static final int backRightRotationMotorID = 0;

        public static final int frontLeftRotationEncoderID = 0;
        public static final int frontRightRotationEncoderID = 0;
        public static final int backLeftRotationEncoderID = 0;
        public static final int backRightRotationEncoderID = 0;

        public static final double frontLeftAngleOffset = 0.0;
        public static final double frontRightAngleOffset = 0.0;
        public static final double backLeftAngleOffset = 0.0;
        public static final double backRightAngleOffset = 0.0;

		public static final double autoDrivePercent = 0.6;
        public static final int driveWheelGearReduction = 0;
        public static final int wheelRadiusM = 0;

        public static final SwerveDriveKinematics kinematics = 
            new SwerveDriveKinematics(
                new Translation2d(trackWidth / 2.0, wheelBase / 2.0), //front left
                new Translation2d(trackWidth / 2.0, -wheelBase / 2.0), //front right
                new Translation2d(-trackWidth / 2.0, wheelBase / 2.0), //rear left
                new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0) //rear right
        );

        public static final SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(0.24, 2.185);

        public static final int pigeonIMU = 0;
        
        public static final double driveJoystickDeadbandPercent = 0;

        public static final double maxDriveSpeed = 0;
        public static final double maxTurnRate = 0;

        public static final double[] driveKps = {0.7, 0.4, 0.7, 0.7};
        public static final double[] driveKds = {3.5, 2.5, 3.7, 3.5};

        public static final double[] rotationKps = {7, 7, 7, 7};
        public static final double[] rotationKds = {0, 0, 0, 0};
        
    }


}
