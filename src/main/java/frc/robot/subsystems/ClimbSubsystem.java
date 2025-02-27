
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class ClimbSubsystem extends SubsystemBase {
  // Motor controller
  private final SparkFlex climbMotor;
  
  // Current commanded speed
  private double currentSpeed = 0.0;
  
  /** Creates a new ClimbSubsystem. */
  public ClimbSubsystem() {
     // Initialize motor controller
    climbMotor = new SparkFlex(ClimbConstants.kClimbMotorCanId, MotorType.kBrushless);
    
    // Create configuration object
    SparkFlexConfig config = new SparkFlexConfig();
    
    // Configure motor parameters
    config.idleMode(IdleMode.kBrake);
    config.smartCurrentLimit(40);
    
    // Apply configuration to the motor controller
    climbMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }
  
  /**
   * Sets the speed of the climb motor
   * @param speed The speed to set (-1.0 to 1.0)
   */
  public void setSpeed(double speed) {
    // Limit speed to valid range
    speed = Math.max(-1.0, Math.min(1.0, speed));
    
    currentSpeed = speed;
    climbMotor.set(speed);
  }
  
  /**
   * Stops the climb motor
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
    
    // Send telemetry to SmartDashboard
    SmartDashboard.putNumber("Climb Speed", currentSpeed);
    SmartDashboard.putNumber("Climb Current", climbMotor.getOutputCurrent());
  }
}