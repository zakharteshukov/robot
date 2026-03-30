package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeederSubsystem;

public class FeederCmd extends Command {
  FeederSubsystem FeederMotor;
  double set;
  public FeederCmd(FeederSubsystem FeederMotor, double set) {
    this.FeederMotor = FeederMotor;
    this.set = set;
    addRequirements(FeederMotor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    FeederMotor.FeedMotorSet(set);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    FeederMotor.FeedMotorSet(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
