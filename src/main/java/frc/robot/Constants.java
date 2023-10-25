package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Constants {

    public final class ArmConstants {

        public static final int wristMotorID = 19;  //19?
        public static final int leftIntakeMotorID = 11;
        public static final int rightIntakeMotorID = 10;
        public static final int topIntakeMotorID = 9;

        public static final double wristkP = 0.1;
        public static final double wristkD = 0.0;

        public static final double intakePosition = -35;
        public static final double upperShootPosition = -3;
        public static final double stowShootPosition = 0;
        public static final double lowShootVoltage = -0.1;
        public static final double midShootVoltage = -0.2;
        public static final double highShootVoltage = -0.75;
        public static final double intakeVoltage = 0.2;
        public static final double spitVoltage = 0.5;

        public static final double topShootPower = 0.5;
        public static final double topIntakePower = -0.75;

    }

    public static final class DriveConstants {

        public static final double trackWidth = Units.inchesToMeters(20.4); // TODO: Might have to change
        public static final double wheelBase = Units.inchesToMeters(22.4); // TODO: Might have to change

        public static final int frontLeftDriveID = 1;
        public static final int frontRightDriveID = 3;
		public static final int backRightDriveID = 5;
		public static final int backLeftDriveID = 7;
		
        public static final int frontLeftRotationMotorID = 2;
        public static final int frontRightRotationMotorID = 4;
        public static final int backRightRotationMotorID = 6;
        public static final int backLeftRotationMotorID = 8;

        public static final int frontLeftRotationEncoderID = 1; 
        public static final int frontRightRotationEncoderID = 2; 
        public static final int backRightRotationEncoderID = 3;
        public static final int backLeftRotationEncoderID = 4; 

        public static final double frontLeftAngleOffset = -2.3930100291016;
        public static final double frontRightAngleOffset = 2.560213934981135; 
        public static final double backRightAngleOffset = 2.389942067525829; // TODO: Might have to switch these
        public static final double backLeftAngleOffset = -0.582912699396544; // TODO: Might have to switch these

		public static final double autoDrivePercent = 0.6;
        public static final double driveWheelGearReduction = 6.12; // placeholder
        public static final double wheelRadiusM = Units.inchesToMeters(2);

        public static final SwerveDriveKinematics kinematics = 
            new SwerveDriveKinematics(
                new Translation2d(trackWidth / 2.0, wheelBase / 2.0), //TODO: Test
                new Translation2d(trackWidth / 2.0, -wheelBase / 2.0), //TODO: Test
                new Translation2d(-trackWidth / 2.0, wheelBase / 2.0), //TODO: Test
                new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0) //TODO: Test
        );

        public static final SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(0.2, 0.3); // TODO: Not used

        public static final int pigeonIMU = 5;
        
        public static final double driveJoystickDeadbandPercent = 0.12;

        public static final double maxDriveSpeed = 5;
        public static final double maxTurnRate = 2 * Math.PI;

        public static final double driveKp = 0.01; //placeholder
        public static final double driveKd = 0.0; //placeholder

        public static final double rotationKp = 7.0;
        public static final double rotationKd = 0.0;
    }


}
