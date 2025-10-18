  // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.CoralCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CoralScoringMechanism;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class InAndOutCoral_CMD extends Command {

  CoralScoringMechanism coralScoringMechanism;
  boolean eject;

  public InAndOutCoral_CMD(CoralScoringMechanism coralScoringMechanism, boolean eject) {

    this.eject = eject;
    this.coralScoringMechanism = coralScoringMechanism;
    addRequirements(coralScoringMechanism);

  }

  @Override
  public void initialize() {
    if(eject)
    {
      coralScoringMechanism.setSpeed(Constants.CoralScoringMechanismConstants.kEjectSpeed);
    }
    else
    {
      coralScoringMechanism.setSpeed(Constants.CoralScoringMechanismConstants.kInsertSpeed);
    }

  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    coralScoringMechanism.setSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
