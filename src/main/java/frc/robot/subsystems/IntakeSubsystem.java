package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeSubsystem {
  private CANSparkMax leftIntakeMotor, rightIntakeMotor, topIntakeMotor;
  private double intakePower;
  private double intakeOff = 0;
  private double intakeOn = ArmConstants.intakeVoltage;
  private final double HARD_STOP = 25.0;

  public IntakeSubsystem() {
    leftIntakeMotor = new CANSparkMax(ArmConstants.leftIntakeMotorID, MotorType.kBrushless);
    rightIntakeMotor = new CANSparkMax(ArmConstants.rightIntakeMotorID, MotorType.kBrushless);
    topIntakeMotor = new CANSparkMax(ArmConstants.topIntakeMotorID, MotorType.kBrushless);
    leftIntakeMotor.follow(rightIntakeMotor, true);
    leftIntakeMotor.setSmartCurrentLimit(80);
    rightIntakeMotor.setSmartCurrentLimit(80);
    topIntakeMotor.setSmartCurrentLimit(80);
  }
  public void intakeOn () {
    intakePower = intakeOn;
  }
  public void intakeOff () {
    intakePower = intakeOff;
  }
  public void setIntakeMotorPower(double percent) {
    rightIntakeMotor.set(percent); // left motor follows?
    topIntakeMotor.set(percent);
  }
  public double getIntakeMotorAmps() {
    return rightIntakeMotor.getOutputCurrent();
  }
  private void checkIntakeAmps() {
    if(getIntakeMotorAmps() > HARD_STOP) {
      setIntakeMotorPower(0);
    }
  }
  @Override
  public void periodic() {
    setIntakeMotorPower(intakeGoalPower);
    intakeOn = SmartDashboard.getNumber("intake power", 0);
    checkIntakeAmps();
  }
}
