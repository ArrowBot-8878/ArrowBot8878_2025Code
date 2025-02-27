// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeIntakeConstants;
import frc.robot.Constants.CoralInsertMechanismConstants;

public class CoralInsertMechanism extends SubsystemBase {
  /** Creates a new CoralMechanism. */
  SparkFlex coralInsertMotor;
  public CoralInsertMechanism() {
    coralInsertMotor = new SparkFlex(CoralInsertMechanismConstants.kCoralInsertMotorCanId, MotorType.kBrushless);

    SparkFlexConfig config = new SparkFlexConfig();
    config.inverted(CoralInsertMechanismConstants.isInverted);
    coralInsertMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  /**
   * Set the speed of the coral insert motor, forward is positive
   * @param speed
   */
  public void setSpeed(double speed) {
    coralInsertMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
