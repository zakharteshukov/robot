package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

//FIXME dönüşüm faktörü eklenecek
public class HoodSubsystem extends SubsystemBase {
  SparkMax hoodMotor;

  SparkMaxConfig hoodConfig;

  SparkClosedLoopController motorPID;
  RelativeEncoder motorEncoder;

  ClosedLoopConfig motorPIDConfig;
  InterpolatingDoubleTreeMap angleTable;

  /**
   * 
   */
  public HoodSubsystem() {
    hoodMotor = new SparkMax(Constants.HoodConstants.HoodMotorID, MotorType.kBrushless);

    hoodConfig = new SparkMaxConfig();

    motorPID = hoodMotor.getClosedLoopController();
    motorEncoder = hoodMotor.getEncoder();

    motorPIDConfig = new ClosedLoopConfig();
    angleTable = new InterpolatingDoubleTreeMap();

    motorPIDConfig.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(
            Constants.HoodConstants.kP,
            Constants.HoodConstants.kI,
            Constants.HoodConstants.kD);

    hoodConfig.encoder.velocityConversionFactor(Constants.HoodConstants.HoodVelConversionF);
    hoodConfig.encoder.positionConversionFactor(Constants.HoodConstants.HoodPosConversionF);
    hoodConfig.idleMode(IdleMode.kBrake);
    hoodConfig.closedLoop.apply(motorPIDConfig);

    // TODO kod yüklendiğinde revden pid değerleri kontrol edilecek
    hoodMotor.configure(hoodConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    resetEncoder();
  }

  public void resetEncoder() {
    motorEncoder.setPosition(0);
  }

  public void setHoodAngle(Angle angle) {
    motorPID.setSetpoint(angle.in(Degrees), ControlType.kPosition);
  }

  public Angle getHoodAngle() {
    return Degrees.of(motorEncoder.getPosition());
  }

  public Angle calculateAngle(double distance) {
    return Degrees.of(angleTable.get(distance));
    // return RPM.of(14.444444 * Math.pow(d, 5) - 224.529075 * Math.pow(d, 4) +
    // 1309.626672 * Math.pow(d, 3) - 3497.041359 * Math.pow(d, 2) + 4241.745837 * d
    // - 44.144144);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("HoodAngle", getHoodAngle().in(Degrees));
  }
}
