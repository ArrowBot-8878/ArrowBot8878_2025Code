// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.AlgaeCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeIntakeSubsystem;

public class AlgaeIntakeBiDirectionalCommand extends Command {
  private final AlgaeIntakeSubsystem intakeSubsystem;
  private final DoubleSupplier speedSupplier;

  /** 
   * Creates a new AlgaeIntakeBiDirectionalCommand.
   * 
   * @param intakeSubsystem The intake subsystem to control
   * @param speedSupplier A supplier for the intake speed (typically from a joystick)
   */
  public AlgaeIntakeBiDirectionalCommand(
      AlgaeIntakeSubsystem intakeSubsystem, 
      DoubleSupplier speedSupplier) {
    this.intakeSubsystem = intakeSubsystem;
    this.speedSupplier = speedSupplier;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Stop the intake motor when the command starts
    intakeSubsystem.setSpeed(0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Get the input from the supplier
    double intakeSpeed = speedSupplier.getAsDouble();
    intakeSubsystem.setSpeed(intakeSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop the intake motor when the command ends
    intakeSubsystem.setSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;  // Run until interrupted
  }
}