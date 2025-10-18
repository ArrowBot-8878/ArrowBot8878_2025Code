// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.CoralCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralArmSubsystem;
import frc.robot.subsystems.CoralWristSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ArmMechGoToPos_CMD extends Command {

  private final CoralArmSubsystem armSubsystem;
  private final CoralWristSubsystem wristSubsystem; 
  private final double armAngle;
  private final double wristAngle;

  public ArmMechGoToPos_CMD(CoralArmSubsystem armSubsystem, CoralWristSubsystem wristSubsystem, double armAngle, double wristAngle) {
    this.armSubsystem = armSubsystem;
    this.wristSubsystem = wristSubsystem;
    this.armAngle = armAngle;
    this.wristAngle = wristAngle;
    addRequirements(armSubsystem, wristSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armSubsystem.setPosition(armAngle);
    wristSubsystem.setPosition(wristAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //FIX ME need to add end logic where if the the endoder is close to the target position then you need to end the command 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Get current positions
    double currentArmAngle = armSubsystem.getAngleDegrees();
    double currentWristAngle = wristSubsystem.getAngleDegrees();
    
    // Calculate absolute differences from target positions
    double armError = Math.abs(currentArmAngle - armAngle);
    double wristError = Math.abs(currentWristAngle - wristAngle);
    
    // Return true if both mechanisms are within 2 degrees of their targets
    return armError <= 2.0 && wristError <= 2.0;
  }
}
