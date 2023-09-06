package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class TankDrive extends CommandBase{

    private final DriveSubsystem drivesub;
    private final Joystick l_Joystick;
    private final Joystick r_Joystick;


    public TankDrive(DriveSubsystem d, Joystick l, Joystick r){
        drivesub = d;
        l_Joystick = l;
        r_Joystick = r;
        addRequirements(drivesub);
    }

    @Override
    public void initialize() {      
    }

    @Override
    public void execute() {
        drivesub.setTankDriveControls(l_Joystick.getX(), r_Joystick.getY());
    }
    
    @Override
    public void end(boolean interrupted) {
    }
}
