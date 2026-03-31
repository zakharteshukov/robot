package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

//FIXME dönüşüm faktörü eklenecek
public class FlyWheelSubsystem extends SubsystemBase {
  SparkMax ShooterMotorLeft;
  SparkMax ShooterMotorRight;

  SparkMaxConfig ShooterConfigLeft;
  SparkMaxConfig ShooterConfigRight;

  SparkClosedLoopController LeftMotorPID;
  RelativeEncoder LeftMotorEncoder;

  ClosedLoopConfig leftMotorPIDConfig;
  FeedForwardConfig LeftMotorFeedForwardConfig;

  InterpolatingDoubleTreeMap rpmTable;

  public FlyWheelSubsystem() {
    ShooterMotorLeft = new SparkMax(Constants.FlyWheelConstants.ShooterMotorLeftID, MotorType.kBrushless);
    ShooterMotorRight = new SparkMax(Constants.FlyWheelConstants.ShooterMotorRightID, MotorType.kBrushless);

    ShooterConfigLeft = new SparkMaxConfig();
    ShooterConfigRight = new SparkMaxConfig();

    LeftMotorPID = ShooterMotorLeft.getClosedLoopController();
    LeftMotorEncoder = ShooterMotorLeft.getEncoder();

    leftMotorPIDConfig = new ClosedLoopConfig();
    rpmTable = new InterpolatingDoubleTreeMap();

    leftMotorPIDConfig.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(
            Constants.FlyWheelConstants.kP,
            Constants.FlyWheelConstants.kI,
            Constants.FlyWheelConstants.kD);

    LeftMotorFeedForwardConfig = new FeedForwardConfig();
    LeftMotorFeedForwardConfig
        .kS(Constants.FlyWheelConstants.kS)
        .kV(Constants.FlyWheelConstants.kV)
        .kA(Constants.FlyWheelConstants.kA);

    ShooterConfigLeft.idleMode(IdleMode.kCoast);
    ShooterConfigLeft.closedLoop.feedForward.apply(LeftMotorFeedForwardConfig);
    ShooterConfigLeft.closedLoop.apply(leftMotorPIDConfig);

    ShooterConfigRight.follow(ShooterMotorLeft, true);
    ShooterMotorLeft.configure(ShooterConfigLeft, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    ShooterMotorRight.configure(ShooterConfigRight, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    rpmTable.put(1.68, -2000.0);
    rpmTable.put(2.17, -1840.0);
    rpmTable.put(2.25, -1540.3);
    rpmTable.put(2.71, -1839.8);
    rpmTable.put(3.03, -1930.1);
    rpmTable.put(3.67, -2097.0);
    rpmTable.put(4.62, -2445.7);
    rpmTable.put(4.15, -2230.6);

    resetEncoder();
  }

  public void resetEncoder() {
    LeftMotorEncoder.setPosition(0);
  }

  public void setFlywheelSpeed(AngularVelocity velocity) {
    LeftMotorPID.setSetpoint(velocity.in(RPM), ControlType.kVelocity);
  }

  public AngularVelocity getFlywheelSpeed() {
    return RPM.of(LeftMotorEncoder.getVelocity());
  }

  public AngularVelocity calculateRpm(double distance) {
    return RPM.of(rpmTable.get(distance));
    // return RPM.of(14.444444 * Math.pow(d, 5) - 224.529075 * Math.pow(d, 4) +
    // 1309.626672 * Math.pow(d, 3) - 3497.041359 * Math.pow(d, 2) + 4241.745837 * d
    // - 44.144144);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("FlywheelSpeed", getFlywheelSpeed().in(RPM));
  }

}