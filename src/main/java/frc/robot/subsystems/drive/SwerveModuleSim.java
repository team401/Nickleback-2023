import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;


public class SwerveModuleSim {
    private final Timer time;
    private double timeSinceUpdate;
    private double simulatedMotorPosition;
    private double simulatedStateSpeed;
    private double lastTimeUpdated;
    private SwerveModuleState simulatedState;
    public SwerveModuleSim() {
        time = new Timer();
        time.start()
        lastTimeUpdated = time.get();
        // initialize state at 0
        simulatedState = new SwerveModuleState(0, Rotation2d.fromDegrees(0));
        simulatedMotorPosition = 0;
        simulatedStateSpeed = 0;
        timeSinceUpdate = 0;
    }

    public void setRotationVoltage(double volts) {
        turnVolts = MathUtil.clamp(volts, -12.0, 12.0);
        turnSim.setInputVoltage(turnVolts)
    }
    public void setDriveVoltage(double volts) {
        driveVolts = MathUtil.clamp(volts, -12.0, 12.0);
        driveSim.setInputVoltage(driveVolts);
    }
}