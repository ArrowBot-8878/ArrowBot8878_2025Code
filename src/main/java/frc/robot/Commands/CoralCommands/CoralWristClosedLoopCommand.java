// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.CoralCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralWristSubsystem;

public class CoralWristClosedLoopCommand extends Command {
  private final CoralWristSubsystem wristSubsystem;
  private final double setpoint;
  
  /**
   * Creates a new CoralWristClosedLoopCommand that sets the wrist position using closed-loop control.
   * 
   * @param wristSubsystem The wrist subsystem to control
   * @param setpoint The target position in rotations
   * @param waitForCompletion Whether to finish the command when the setpoint is reached
   */
  public CoralWristClosedLoopCommand(CoralWristSubsystem wristSubsystem, double setpoint) {
    this.wristSubsystem = wristSubsystem;
    this.setpoint = setpoint;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(wristSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Set the setpoint
    wristSubsystem.setPosition(setpoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Nothing needed here, the PID controller in the subsystem will handle reaching the setpoint
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}