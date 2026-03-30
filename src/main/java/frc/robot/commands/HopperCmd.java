package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HopperSubsystem;

public class HopperCmd extends Command {
  HopperSubsystem hopperSubsystem;
  double set;
  public HopperCmd(HopperSubsystem hopperSubsystem, double set) {
    this.hopperSubsystem = hopperSubsystem;
    this.set= set;
    addRequirements(hopperSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    hopperSubsystem.HopperMotorSet(set);
  }

  @Override
  public void end(boolean interrupted) {
    hopperSubsystem.HopperMotorSet(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
