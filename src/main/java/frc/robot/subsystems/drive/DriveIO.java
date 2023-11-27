package frc.robot.subsystems.drive;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveIO extends SubsystemBase {
    public double getDrivePosition() { return 0.0; }

    public double getRotationPosition() { return 0.0; }

    public double getRotationVelocity() { return 0.0; }

    public double getDriveVelocityMPerS() { return 0.0; }       

    public double getDriveVelocityRadPerS() { return 0.0; }

    public void setRotationVoltage(double volts) {}

    public void setDriveVoltage(double volts) {}

    public void setDriveVelocity(double velocityRadPerS) {}

    public void zeroEncoders() {}

    public void setDrivePD(double p, double d) {}

    public double getDriveStatorCurrent() { return 0.0; }

    public double getRotationStatorCurrent() { return 0.0; }

    public void setBrake(boolean braked) {}

    public SwerveModulePosition getModulePosition() { return null; } 

    public SwerveModuleState getModuleState() { return null; }

    public void setGoalModuleState(SwerveModuleState state) {}

    public void initModulePosition() {}

    public void moduleControl(SwerveModuleState optimizedState) {}

    public void updateModuleSimulation() {}
}
