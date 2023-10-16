package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.ArmSubsystem;

public class Intake {
  private CANSparkMax leftMotor, rightMotor, topIntakeMotor;
  private double intakePower;
  private final double CURRENT_LIMIT = 25.0;
  private boolean runTopIntakeMotor = false;

  public Intake() {
    leftMotor = new CANSparkMax(ArmConstants.leftIntakeMotorID, MotorType.kBrushless);
    rightMotor = new CANSparkMax(ArmConstants.rightIntakeMotorID, MotorType.kBrushless);
    topIntakeMotor = new CANSparkMax(ArmConstants.topIntakeMotorID, MotorType.kBrushless);
    leftMotor.follow(rightMotor, true);
    leftMotor.setSmartCurrentLimit(80);
    rightMotor.setSmartCurrentLimit(80);
    topIntakeMotor.setSmartCurrentLimit(80);
  }

  public void intake () {
    intakePower = ArmConstants.intakeVoltage;
    runTopIntakeMotor = true;
  }

  public void shootLow () {
    intakePower = ArmConstants.lowShootVoltage;
    runTopIntakeMotor = false;
  }

  public void shootMid () {
    intakePower = ArmConstants.midShootVoltage;
    runTopIntakeMotor = false;
  }

  public void shootHigh () {
    intakePower = ArmConstants.highShootVoltage;
    runTopIntakeMotor = false;
  }

  public void spit () {
    intakePower = -ArmConstants.spitVoltage;
    runTopIntakeMotor = true;
  }

  public void off () {
    intakePower = 0;
    runTopIntakeMotor = true;
  }

  public void setIntakeMotorPower(double percent) {
    rightMotor.set(percent);
    if(runTopIntakeMotor == true) {
      topIntakeMotor.set(percent);
    } else {
      topIntakeMotor.set(0);
    }
  }
  public double getIntakeMotorAmps() {
    return rightMotor.getOutputCurrent();
  }
  private void checkIntakeAmps() {
    if(getIntakeMotorAmps() > CURRENT_LIMIT) {
      setIntakeMotorPower(0);
    }
  }
  
  public void run() {
    setIntakeMotorPower(intakePower);
    checkIntakeAmps();
  }
}
