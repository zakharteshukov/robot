package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FeederSubsystem extends SubsystemBase {
  SparkMax FeedMotor;
  SparkMaxConfig FeedConfig;
  public FeederSubsystem() {
    FeedMotor = new SparkMax(Constants.FeederConstants.FeederMotorID, MotorType.kBrushless);
    FeedConfig = new SparkMaxConfig();

    FeedConfig.idleMode(IdleMode.kCoast);
    FeedConfig.inverted(false);
    FeedMotor.configure(FeedConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void FeedMotorSet(double set){
    FeedMotor.set(set);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
