package frc.robot.subsystems.drive;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveAngle extends SubsystemBase{

    private final PigeonIMU pigeon;

    private double degHeadingOffset = 0;

    private double degPitchOffset = 0;

    private double degRollOffset = 0;

    public DriveAngle() {
        pigeon = new PigeonIMU(DriveConstants.pigeonIMU);
        degHeadingOffset = pigeon.getYaw();
        degPitchOffset = pigeon.getPitch();
        degRollOffset = pigeon.getRoll();
    }

    public double getHeading() {
        return Units.degreesToRadians(pigeon.getYaw() - degHeadingOffset);
    }

    public void resetHeading() {
        degHeadingOffset = pigeon.getYaw();
    }

    public void setHeading(double headingRad) {
        degHeadingOffset = pigeon.getYaw() - Units.radiansToDegrees(headingRad);
    }

    public double getPitch() {
        return Units.degreesToRadians(pigeon.getPitch() - degPitchOffset);
    }

    public void resetPitch() {
        degPitchOffset = pigeon.getPitch();
    } 

    public double getRoll() {
        return Units.degreesToRadians(pigeon.getRoll() - degRollOffset);
    }

    public void resetRoll() {
        degRollOffset = pigeon.getRoll();
    } 

    
}
