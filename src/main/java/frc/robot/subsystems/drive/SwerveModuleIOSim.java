import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.math.system.plant.DCMotor;

public class SwerveModuleIOSim extends SubsystemBase {
    private FlywheelSim driveSim = new FlywheelSim(DCMotor.getNEO(1), /* gearing, jKgMetersSquared */);
    private FlywheelSim turnSim = new FlywheelSim(DCMotor.getNEO(1), /* gearing, jKgMetersSquared */);
    private double relativeTurnPositionRad = 0.0;
    private double absoluteTurnPositionRad = Math.random() * 2.0 * Math.PI;
    private double driveVolts = 0.0;
    private double turnVolts = 0.0;
}