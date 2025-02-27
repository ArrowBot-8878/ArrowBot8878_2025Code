// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeIntakeConstants;

public class AlgaeIntakeSubsystem extends SubsystemBase {
  /** Creates a new AlgaeIntake. */
  private final SparkFlex intakeMotor;
  private double currentSpeed = 0.0;

  public AlgaeIntakeSubsystem() {
    intakeMotor = new SparkFlex(AlgaeIntakeConstants.kAlgaeIntakeMotorCanId, MotorType.kBrushless);
  }

  /**
   * Set the speed of the algae intake motor, forward is positive
   * @param speed The speed to set (-1.0 to 1.0)
   */
  public void setSpeed(double speed) {
    currentSpeed = speed;
    intakeMotor.set(speed);
  }

  /**
   * Stops the intake motor
   */
  public void stop() {
    setSpeed(0.0);
  }

  /**
   * Gets the current commanded speed
   * @return Current speed (-1.0 to 1.0)
   */
  public double getCurrentSpeed() {
    return currentSpeed;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Algae Intake Speed", currentSpeed);
  }
}