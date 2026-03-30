package frc.robot.subsystems;
import static edu.wpi.first.units.Units.Degrees;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

//FIXME dönüşüm faktörü eklenecek
public class IntakeSubsystem extends SubsystemBase {
  SparkMax IntakeMotorLeft;
  SparkMax IntakeMotorRight;

  SparkMaxConfig IntakeConfigLeft;
  SparkMaxConfig IntakeConfigRight;
  
  SparkClosedLoopController LeftMotorPID;
  RelativeEncoder LeftMotorEncoder; 

  ClosedLoopConfig leftMotorPIDConfig;
  public IntakeSubsystem() {
    IntakeMotorLeft = new SparkMax(Constants.IntakeConstants.IntakeMotorLeftID, MotorType.kBrushless);
    IntakeMotorRight = new SparkMax(Constants.IntakeConstants.IntakeMotorRightID, MotorType.kBrushless);

    IntakeConfigLeft = new SparkMaxConfig();
    IntakeConfigRight = new SparkMaxConfig();

    LeftMotorPID = IntakeMotorLeft.getClosedLoopController();
    LeftMotorEncoder = IntakeMotorLeft.getEncoder();
    
    leftMotorPIDConfig = new ClosedLoopConfig();
    leftMotorPIDConfig.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .pid(
      Constants.IntakeConstants.kP, 
      Constants.IntakeConstants.kI, 
      Constants.IntakeConstants.kD
    ).outputRange(0.3, -0.3);

    IntakeConfigLeft.encoder.velocityConversionFactor(Constants.IntakeConstants.IntakeVelConversionF);
    IntakeConfigLeft.encoder.positionConversionFactor(Constants.IntakeConstants.IntakePosConversionF);
    IntakeConfigLeft.idleMode(IdleMode.kBrake);
    IntakeConfigLeft.closedLoop.apply(leftMotorPIDConfig);
    IntakeConfigRight.apply(IntakeConfigLeft);
    IntakeConfigRight.follow(IntakeMotorLeft, true); //TODO gearboxa göre kontrol edilecek
    IntakeMotorLeft.configure(IntakeConfigLeft, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    IntakeMotorRight.configure(IntakeConfigRight, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }
  
  public void setIntakeAngle(Angle angle) {
    LeftMotorPID.setSetpoint(angle.in(Degrees), ControlType.kPosition);
  }

  public void setIntakeMotor(double set) {
    IntakeMotorLeft.set(set);
  }

  public Angle getIntakeAngle() {
    return Degrees.of(LeftMotorEncoder.getPosition());
  }
  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("IntakeAngle", getIntakeAngle().in(Degrees));
  }
}
