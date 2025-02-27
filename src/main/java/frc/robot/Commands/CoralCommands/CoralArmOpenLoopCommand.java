// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.CoralCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralArmSubsystem;

public class CoralArmOpenLoopCommand extends Command {
  private final CoralArmSubsystem armSubsystem;
  private final DoubleSupplier speedSupplier;

  /**
   * Creates a new CoralArmOpenLoopCommand that sets the arm speed based on a supplier.
   * 
   * @param armSubsystem The arm subsystem to control
   * @param speedSupplier A supplier that provides the speed value (-1.0 to 1.0)
   */
  public CoralArmOpenLoopCommand(CoralArmSubsystem armSubsystem, DoubleSupplier speedSupplier) {
    this.armSubsystem = armSubsystem;
    this.speedSupplier = speedSupplier;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(armSubsystem);
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
    armSubsystem.setSpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop the arm motor when the command ends
    armSubsystem.setSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; // Run until interrupted
  }
}