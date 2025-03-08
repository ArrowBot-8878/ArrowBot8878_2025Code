// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeIntakeConstants;
import frc.robot.Constants.CoralScoringMechanismConstants;
import frc.robot.Constants.CoralScoringMechanismConstants;

public class CoralScoringMechanism extends SubsystemBase {
  /** Creates a new CoralMechanism. */
  SparkMax CoralMotor1;
  SparkMax CoralMotor2;
  public CoralScoringMechanism() {
    CoralMotor1 = new SparkMax(CoralScoringMechanismConstants.kCoralMotor1CanId, MotorType.kBrushless);
    CoralMotor2 = new SparkMax(CoralScoringMechanismConstants.kCoralMotor2CanId, MotorType.kBrushless);


    SparkMaxConfig config = new SparkMaxConfig();
    config.inverted(CoralScoringMechanismConstants.isInverted);
    CoralMotor1.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    CoralMotor2.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  /**
   * Set the speed of the coral insert motor, forward is positive
   * @param speed
   */
  public void setSpeed(double speed) {
    CoralMotor1.set(speed);
    CoralMotor2.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
