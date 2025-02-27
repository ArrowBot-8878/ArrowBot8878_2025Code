// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.ClimbCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbBiDirectionalCommand extends Command {
  private final ClimbSubsystem climbSubsystem;
  private final DoubleSupplier speedSupplier;

  /** 
   * Creates a new ClimbBiDirectionalCommand.
   * 
   * @param climbSubsystem The climb subsystem to control
   * @param speedSupplier A supplier for the climb speed (typically from a joystick)
   */
  public ClimbBiDirectionalCommand(
      ClimbSubsystem climbSubsystem, 
      DoubleSupplier speedSupplier) {
    this.climbSubsystem = climbSubsystem;
    this.speedSupplier = speedSupplier;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climbSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Stop the climb motor when the command starts
    climbSubsystem.setSpeed(0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Get the input from the supplier
    double climbSpeed = speedSupplier.getAsDouble();
    climbSubsystem.setSpeed(climbSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop the climb motor when the command ends
    climbSubsystem.setSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;  // Run until interrupted
  }
}