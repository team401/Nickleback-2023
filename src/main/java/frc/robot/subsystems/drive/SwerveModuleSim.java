import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;


public class SwerveModuleSim {
    private final Timer time;
    private double timeSinceUpdate;
    private double simulatedPosition;
    private double simulatedStateSpeed;
    private double lastTimeUpdated;
    private SwerveModuleState simulatedState;
    public SwerveModuleSim() {
        time = new Timer();
        time.start()
        lastTimeUpdated = time.get();
        // initialize state at 0
        simulatedState = new SwerveModuleState(0, Rotation2d.fromDegrees(0));
        simulatedPosition = 0;
        simulatedStateSpeed = 0;
        timeSinceUpdate = 0;
    }
    /*
        @param desiredState - state swerve module should be set to
    */
    public void updateStateAndPosition(SwerveModuleState desiredState) {
        timeSinceUpdate = timer.get() - lastTime;
        lastTime = timer.get();

        simulatedState = desiredState;
        simulatedStateSpeed = simulatedState.speedMetersPerSecond;
        simulatedPosition += (simulatedStateSpeed * timeSinceUpdate);
    }
    /*
        @return SwerveModulePosition of simulatedState
    */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(simulatedPosition, simulatedState.angle);
    }
    /*
        @return the simulated state
    */
    public SwerveModuleState getState() {
        return simulatedState;
    }
}