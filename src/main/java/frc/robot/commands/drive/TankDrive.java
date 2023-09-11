package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class TankDrive extends CommandBase{

    private final DriveSubsystem drivesub;

    private final DoubleSupplier xPercent;
    private final DoubleSupplier yPercent;

    public TankDrive(DriveSubsystem d, DoubleSupplier xPcent, DoubleSupplier yPcent){
        drivesub = d;
        xPercent = xPcent;
        yPercent = yPcent;
        addRequirements(drivesub);
    }

    @Override
    public void initialize() {      
    }

    @Override
    public void execute() {
        drivesub.setTankDriveControls(xPercent.getAsDouble(), yPercent.getAsDouble());
    }
    
    @Override
    public void end(boolean interrupted) {
    }
}
