package frc.robot.subsystems.drive;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ChassisMode;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveMode;
import frc.robot.RobotState;

public class DriveSubsystem extends SubsystemBase{


    private final SwerveModules[] driveModules = new SwerveModules[4];
    private final DriveAngle driveAngle = new DriveAngle();
    private SwerveModulePosition modulePositions[] = new SwerveModulePosition[4];
    private SwerveModuleState[] goalModuleStates = new SwerveModuleState[4];
    private PIDController[] rotationPIDs = new PIDController[4];

    private DriveMode mode;

    private DriveHardware driveHardware;

    private double leftTank = 0;
    private double rightTank = 0;
    private double arcadeForward = 0;
    private double arcadeRotation = 0;
    
    private boolean babyMode = false;


    /**
     * @param chassisMode
     * @param driveMode
     */
    public DriveSubsystem(ChassisMode chassisMode, DriveMode driveMode) {
        if (chassisMode == ChassisMode.B_TEAM) {
            driveHardware = new DriveSparkMAX(DriveConstants.frontLeftID, DriveConstants.frontRightID, DriveConstants.backLeftID, DriveConstants.backRightID);
        } else {
            driveHardware = new DriveVictor(DriveConstants.frontLeftID, DriveConstants.frontRightID, DriveConstants.backLeftID, DriveConstants.backRightID);
        }
        mode = driveMode;

        driveModules[0] = new SwerveModules(DriveConstants.frontLeftID, DriveConstants.frontLeftRotationMotorID,
        DriveConstants.frontLeftRotationEncoderID, DriveConstants.frontLeftAngleOffset, false, false);
        driveModules[1] = new SwerveModules(DriveConstants.frontRightID, DriveConstants.frontRightRotationMotorID,
        DriveConstants.frontRightRotationEncoderID, DriveConstants.frontRightAngleOffset, true, false);
        driveModules[2] = new SwerveModules(DriveConstants.backLeftID, DriveConstants.backLeftRotationMotorID,
        DriveConstants.backLeftRotationEncoderID, DriveConstants.backLeftAngleOffset, false, false);
        driveModules[3] = new SwerveModules(DriveConstants.backRightID, DriveConstants.backRightRotationMotorID,
        DriveConstants.backRightRotationEncoderID, DriveConstants.backRightAngleOffset, true, false);

        for (int i = 0; i < 4; i++) {
            driveModules[i].zeroEncoders();
            modulePositions[i] = new SwerveModulePosition();
            goalModuleStates[i] = new SwerveModuleState();

            rotationPIDs[i] = new PIDController(DriveConstants.rotationKps[i], 0, DriveConstants.rotationKds[i]);
            rotationPIDs[i].enableContinuousInput(-Math.PI, Math.PI);
            driveModules[i].setDrivePD(DriveConstants.driveKps[i], DriveConstants.driveKds[i]);

            modulePositions[i].distanceMeters = driveModules[i].getDrivePosition() * DriveConstants.wheelRadiusM;
            modulePositions[i].angle = new Rotation2d(driveModules[i].getRotationPosition());
        }

        setGoalChassisSpeeds(new ChassisSpeeds(0, 0, 0));

        RobotState.getInstance();//.initializePoseEstimator(getRotation(), modulePositions);
    }

    

    public DriveSubsystem(ChassisMode chassisMode) {
        this(chassisMode, DriveMode.TANKDRIVE);
    }

    public DriveSubsystem() {
        this(ChassisMode.C_TEAM, DriveMode.TANKDRIVE);
    }
    

    public void setArcadeDriveControls(double forward, double rotation) {
        mode = DriveMode.ARCADEDRIVE;
		arcadeForward = forward;
        arcadeRotation = rotation;
	}

	public void setTankDriveControls(double left, double right) {
        mode = DriveMode.TANKDRIVE;
        leftTank = left;
        rightTank = right;
	}

    public void swapMode() {
        if(mode == DriveMode.TANKDRIVE) {
            mode = DriveMode.ARCADEDRIVE;
            setArcadeDriveControls(arcadeForward, arcadeRotation);
        } else {
            mode = DriveMode.TANKDRIVE;
            setTankDriveControls(leftTank, rightTank);
        }
    }

    public DriveMode getMode(){
        return mode;
    }

    @Override
    public void periodic(){
        if (mode == DriveMode.TANKDRIVE){
            driveHardware.tankDrive(leftTank, rightTank);
        }
        else {
            driveHardware.arcadeDrive(arcadeForward, arcadeRotation);
        }

        for (int i = 0; i < 4; i++) {

            //SmartDashboard.putBoolean("Drive/modules/"+i+"/status", driveModules[i].getDeadBoolean());

            Rotation2d moduleRotation = new Rotation2d(driveModules[i].getRotationPosition());

            SwerveModuleState optimizedState = SwerveModuleState.optimize(goalModuleStates[i], moduleRotation);
            double rotationSetpointRadians = optimizedState.angle.getRadians();
            double speedSetpointMPerS = optimizedState.speedMetersPerSecond;

            double speedRadPerS = speedSetpointMPerS / DriveConstants.wheelRadiusM;
            double ffVolts = DriveConstants.driveFF.calculate(speedRadPerS);
            driveModules[i].setDriveVelocity(speedRadPerS, ffVolts);

            double rotationVoltage = rotationPIDs[i].calculate(moduleRotation.getRadians(), rotationSetpointRadians);
            driveModules[i].setRotationVoltage(rotationVoltage);

        }

        for (int i = 0; i < 4; i++) {
            modulePositions[i].distanceMeters = driveModules[i].getDrivePosition() * DriveConstants.wheelRadiusM;
            modulePositions[i].angle = new Rotation2d(driveModules[i].getRotationPosition());

            SmartDashboard.putNumber("Drive/modules/"+i+"/angle", modulePositions[i].angle.getRadians());
            SmartDashboard.putNumber("Drive/modules/"+i+"/drive stator current", driveModules[i].getDriveStatorCurrent());
            SmartDashboard.putNumber("Drive/modules/"+i+"/rotation stator current", driveModules[i].getRotationStatorCurrent());
        }
        RobotState.getInstance().recordDriveObservations(/*getRotation(), modulePositions*/);

        SmartDashboard.putNumber("Drive/velocity magnitude", getChassisSpeeds().vxMetersPerSecond);
    }


    public void setGoalModuleStates(SwerveModuleState[] states) {
        for (int i = 0; i < 4; i++) {
            goalModuleStates[i] = states[i];
        }
    }

    public void setGoalChassisSpeeds(ChassisSpeeds speeds) {
        speeds = new ChassisSpeeds(speeds.vxMetersPerSecond * (babyMode ? 0.2 : 1), speeds.vyMetersPerSecond * (babyMode ? 0.2 : 1), speeds.omegaRadiansPerSecond * (babyMode ? 0.1 : 1));
        SwerveModuleState[] goalModuleStates = DriveConstants.kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(goalModuleStates, DriveConstants.maxDriveSpeed);
        if (speeds.vxMetersPerSecond == 0 && speeds.vyMetersPerSecond == 0 && speeds.omegaRadiansPerSecond == 0) {
            goalModuleStates = new SwerveModuleState[] {
                new SwerveModuleState(0, modulePositions[0].angle), 
                new SwerveModuleState(0, modulePositions[1].angle),
                new SwerveModuleState(0, modulePositions[2].angle),
                new SwerveModuleState(0, modulePositions[3].angle)
            };
        }
        setGoalModuleStates(goalModuleStates);
    }

    public Rotation2d getRotation() {
        return new Rotation2d(MathUtil.angleModulus(driveAngle.getHeading()));
    }

        public void resetHeading() {
        driveAngle.resetHeading();
    }

    public void setHeading(double heading) {
        driveAngle.setHeading(heading);
    }

    public double getRoll() {
        return driveAngle.getRoll();
    }

    public void setBabyMode(boolean baby) {
        babyMode = baby;
    }

    public void setFieldToVehicle(Pose2d fieldToVehicle) {
        RobotState.getInstance().setFieldToVehicle(/*getRotation(), modulePositions, fieldToVehicle*/);
    }

    public void setVolts(double v) {
        driveModules[0].setDriveVoltage(v);
        driveModules[1].setDriveVoltage(v);
        driveModules[2].setDriveVoltage(v);
        driveModules[3].setDriveVoltage(v);
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = driveModules[i].getModuleState();
        }
        return states;
    }

    public ChassisSpeeds getChassisSpeeds() {
        return DriveConstants.kinematics.toChassisSpeeds(getModuleStates());
    }

    public void setBrakeMode(boolean braked) {
        for (int i = 0; i < 4; i++) {
            driveModules[i].setBrake(braked);
        }
    }

} 


