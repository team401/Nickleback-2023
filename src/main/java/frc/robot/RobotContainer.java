package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.drive.ArcadeDrive;
import frc.robot.commands.drive.TankDrive;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.arm.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.arm.ArmSubsystem;import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.ButtonMasher;
import frc.robot.Thrustmaster;

public class RobotContainer extends SubsystemBase{
    private final DriveSubsystem drive = new DriveSubsystem();

    private final Joystick leftStick = new Joystick(0);
    private final Joystick rightStick = new Joystick(1);
    private final XboxController gamepad = new XboxController(2);

    private ArmSubsystem wrist = new ArmSubsystem();

    public RobotContainer(){

        drive.setDefaultCommand(new TankDrive(drive, 
        () -> leftStick.getRawAxis(1),
        () -> rightStick.getRawAxis(0)));

    }


    
    private ArmSubsystem wrist = new ArmSubsystem();
    //private XboxController gamepad = new XboxController(0);

    private final Thrustmaster leftStick = new Thrustmaster(0);
    private final Thrustmaster rightStick = new Thrustmaster(1);
    private final ButtonMasher masher = new ButtonMasher(2);

    SendableChooser<String> autoChooser = new SendableChooser<String>();
    private Command activeAutoCommand = null;
    private String activeAutoName = null;
    }

    public RobotContainer() {

        configureSubsystems();
        configureCompBindings();
        // configureTestBindings();
    }

    private void configureCompBindings() {

    }

    private void configureSubsystems() {

        drive.setDefaultCommand(new DriveWithJoysticks(
            drive,
            () -> -leftStick.getRawAxis(1),
            () -> -leftStick.getRawAxis(0),
            () -> -rightStick.getRawAxis(0),
            true
        ));

    }
}
