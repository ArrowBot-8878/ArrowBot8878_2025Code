// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.AlgaeCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeWristSubsystem;

public class AlgaeWristOpenLoopCommand extends Command {
  private final AlgaeWristSubsystem wristSubsystem;
  private final DoubleSupplier speedSupplier;

  /**
   * Creates a new AlgaeWristOpenLoopCommand that sets the wrist speed based on a supplier.
   * 
   * @param wristSubsystem The wrist subsystem to control
   * @param speedSupplier A supplier that provides the speed value (-1.0 to 1.0)
   */
  public AlgaeWristOpenLoopCommand(AlgaeWristSubsystem wristSubsystem, DoubleSupplier speedSupplier) {
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