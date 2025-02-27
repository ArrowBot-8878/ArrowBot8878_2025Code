package frc.robot.Commands.CoralCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralWristSubsystem;

public class CoralWristOpenLoopCommand extends Command {
  private final CoralWristSubsystem wristSubsystem;
  private final DoubleSupplier speedSupplier;

  /**
   * Creates a new CoralWristOpenLoopCommand that sets the wrist speed based on a supplier.
   * 
   * @param wristSubsystem The wrist subsystem to control
   * @param speedSupplier A supplier that provides the speed value (-1.0 to 1.0)
   */
  public CoralWristOpenLoopCommand(CoralWristSubsystem wristSubsystem, DoubleSupplier speedSupplier) {
    this.wristSubsystem = wristSubsystem;
    this.speedSupplier = speedSupplier;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(wristSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Nothing needed here, we'll set speed in execute()
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Get the current speed from the supplier and apply it
    double speed = speedSupplier.getAsDouble();
    wristSubsystem.setSpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop the wrist motor when the command ends
    wristSubsystem.setSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; // Run until interrupted
  }
}