package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.NeoDriveSubsystem;

public class NeoTankDrive extends CommandBase {

    private final NeoDriveSubsystem neoDriveSubsystem;
    private final DoubleSupplier leftPower;
    private final DoubleSupplier rightPower;

    public NeoTankDrive(NeoDriveSubsystem d, DoubleSupplier l, DoubleSupplier r) {
        this.neoDriveSubsystem = d;
        this.leftPower = l;
        this.rightPower = r;
        addRequirements(neoDriveSubsystem);
    }

    @Override
    public void initialize() {      
    }

    @Override
    public void execute() {
        this.neoDriveSubsystem.setTankDriveControls(this.leftPower.getAsDouble(), this.rightPower.getAsDouble());
    }
    
    @Override
    public void end(boolean interrupted) {
    }
}
