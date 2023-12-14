package frc.robot;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ArmMove;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.Mode;
import frc.robot.subsystems.drive.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;


public class RobotContainer {

    private final DriveSubsystem drive = new DriveSubsystem();
    //private ArmSubsystem arm = new ArmSubsystem();

    private Joystick leftJoystick = new Joystick(0);
    private Joystick rightJoystick = new Joystick(1);

    private CommandXboxController gamepad = new CommandXboxController(2);

    /*private ArmMove armIntake = new ArmMove(arm, Mode.INTAKE);
    private ArmMove armShootHigh = new ArmMove(arm, Mode.SHOOT_HIGH);
    private ArmMove armShootMid = new ArmMove(arm, Mode.SHOOT_MID);
    private ArmMove armShootLow = new ArmMove(arm, Mode.SHOOT_LOW);
    private ArmMove armSpit = new ArmMove(arm, Mode.SPIT);*/

    SendableChooser<String> autoChooser = new SendableChooser<String>();
    

    public RobotContainer() {
        SmartDashboard.putNumber("running", 0);
        
        drive.setDefaultCommand(new DriveWithJoysticks(
            drive,
            () -> -leftJoystick.getRawAxis(1),
            () -> -leftJoystick.getRawAxis(0),
            () -> -rightJoystick.getRawAxis(0),
            true
        ));
        configureButtonBindings();
        
        configAuto();

        
    }

    private void configureButtonBindings() {
        /*gamepad.a()
            .whileTrue(armIntake);
        gamepad.y()
            .whileTrue(armShootHigh);
        gamepad.b()
            .whileTrue(armShootMid);
        gamepad.x()
            .whileTrue(armShootLow);
        new Trigger(() -> gamepad.getLeftY() < -0.7)
            .whileTrue(armSpit);*/
        new JoystickButton(rightJoystick, 1)
            .whileTrue(new InstantCommand(
                () -> {drive.resetHeading();
                }));
            
    }

    public Command getAutonomous() {

        //return new PathPlannerAuto("Testing Auto");
        return new PathPlannerAuto(autoChooser.getSelected());
        
    }

    private void configAuto() {

        autoChooser.setDefaultOption("Position 1, 2 Ball, No Balance", "1-2_Cube");
        autoChooser.addOption("Position 1, 1 Ball, Balance", "1-1_Cube_Balance");
        autoChooser.addOption("Testing Path", "Testing Auto");

        SmartDashboard.putData("Auto Mode", autoChooser);


        
        List<Pair<String, Command>> commands = new ArrayList<Pair<String, Command>>();
        /*commands.add(new Pair<String, Command> ("shootCube", 
            new SequentialCommandGroup(
                new RunCommand(() -> arm.setMode(Mode.SHOOT_HIGH)),
                new WaitCommand(0.1),
                new RunCommand(() -> arm.setMode(Mode.STOW))
            )));

        commands.add(new Pair<String, Command> ("intakeCube", 
            new SequentialCommandGroup(
                new RunCommand(() -> arm.setMode(Mode.INTAKE)),
                new WaitCommand(1),
                new RunCommand(() -> arm.setMode(Mode.STOW))
            )));

        */

        SmartDashboard.putBoolean("shootTime", false);
        SmartDashboard.putBoolean("intakeTime", false);

        commands.add(new Pair<String, Command> ("shootCube", 
            new InstantCommand(() -> {
                SmartDashboard.putBoolean("shootTime", true);
                SmartDashboard.putBoolean("intakeTime", false);})));

       commands.add(new Pair<String, Command> ("intakeCube", 
            new InstantCommand(() -> {
                SmartDashboard.putBoolean("intakeTime", true);
                SmartDashboard.putBoolean("shootTime", false);})));
    
        NamedCommands.registerCommands(commands);
    }



}
