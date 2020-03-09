package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {

  Joystick joystick = new Joystick(0);
  XboxController xbox = new XboxController(2);

  CANSparkMax leftDrive1 = new CANSparkMax(1, MotorType.kBrushless);
  CANSparkMax leftDrive2 = new CANSparkMax(2, MotorType.kBrushless);
  CANSparkMax rightDrive1 = new CANSparkMax(3, MotorType.kBrushless);
  CANSparkMax rightDrive2 = new CANSparkMax(4, MotorType.kBrushless);
  CANSparkMax turret = new CANSparkMax(5, MotorType.kBrushless);
  CANSparkMax shooter1 = new CANSparkMax(6, MotorType.kBrushless);
  CANSparkMax shooter2 = new CANSparkMax(7, MotorType.kBrushless);

  VictorSP rollerMotor = new VictorSP(0);
  VictorSP serializerMotor = new VictorSP(1);
  VictorSP ballTubeMotor = new VictorSP(2);
  VictorSP hoodMotor = new VictorSP(6);
  VictorSP hopupMotor = new VictorSP(8);

  Encoder hoodEncoder = new Encoder(0, 1);
  Encoder turretEncoder = new Encoder(2, 3);
  Encoder ballTubeEncoder = new Encoder(4, 5);

  DigitalInput linebreakIn = new DigitalInput(9);
  DigitalInput linebreakOut = new DigitalInput(8);

  DoubleSolenoid OTBSolenoid = new DoubleSolenoid(0, 7);
  DoubleSolenoid shifterSolenoid = new DoubleSolenoid(1, 6);

  DifferentialDrive drivetrain;

  PIDController controller = new PIDController(0, 0, 0);

  String[] numberInputs = {
    "turretPower",
    "shooterPower",
    "rollerPower",
    "serializerPower",
    "ballTubePower",
    "hoodPower",
    "hopupPower",
    "shooterVeloTarget",
    "shooterP",
    "shooterF"
  };

  @Override
  public void robotInit() {
    leftDrive2.follow(leftDrive1);
    rightDrive2.follow(rightDrive1);
    drivetrain = new DifferentialDrive(leftDrive1, rightDrive1);

    for(String name : numberInputs){
      SmartDashboard.putNumber(name, 0);
    }

    SmartDashboard.putBoolean("OTBSolenoid", false);
    SmartDashboard.putBoolean("shifterSolenoid", false);
  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("hoodEncoder", hoodEncoder.get());
    SmartDashboard.putNumber("turretEncoder", turretEncoder.get());
    SmartDashboard.putNumber("ballTubeEncoder", ballTubeEncoder.get());
    SmartDashboard.putBoolean("linebreakIn", linebreakIn.get());
    SmartDashboard.putBoolean("linebreakOut", linebreakOut.get());
  }

  @Override
  public void teleopPeriodic() {
    drivetrain.arcadeDrive(-joystick.getY(), joystick.getX());

    turret.set(SmartDashboard.getNumber("turretPower", 0));
    shooter1.set(SmartDashboard.getNumber("shooterPower", 0));
    rollerMotor.set(SmartDashboard.getNumber("rollerPower", 0));
    serializerMotor.set(SmartDashboard.getNumber("serializerPower", 0));
    ballTubeMotor.set(SmartDashboard.getNumber("ballTubePower", 0));
    hoodMotor.set(SmartDashboard.getNumber("hoodPower", 0));
    hopupMotor.set(SmartDashboard.getNumber("hopupPower", 0));

    OTBSolenoid.set(SmartDashboard.getBoolean("OTBSolenoid", false) ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
    shifterSolenoid.set(SmartDashboard.getBoolean("shifterSolenoid", false) ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);

    controller.setP(SmartDashboard.getNumber("shooterP", 0));
    double target = SmartDashboard.getNumber("shooterVeloTarget", 0);
    double feedforward = SmartDashboard.getNumber("shooterF", 0) * target;
    double PIDpower = controller.calculate(shooter1.getEncoder().getVelocity(), target);
    SmartDashboard.putNumber("shooterPower", feedforward + PIDpower);
  }
}