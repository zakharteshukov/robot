package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeRollerSubsystem;

public class IntakeRollerCmd extends Command {
  IntakeRollerSubsystem intakeRoller;
  double set;
  public IntakeRollerCmd(IntakeRollerSubsystem IntakeRoller, double set) {
    this.intakeRoller = IntakeRoller;
    this.set = set;
    addRequirements(IntakeRoller);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeRoller.IntakeRollerSet(set);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeRoller.IntakeRollerSet(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
