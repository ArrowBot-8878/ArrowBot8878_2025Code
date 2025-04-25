// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AlgaeIntakeConstants;
import frc.robot.Constants.CoralArmConstants;

public class CoralArmSubsystem extends SubsystemBase {
  // Motor controller
  private final SparkFlex armMotor;
  private final SparkMax armBore;
  private final AbsoluteEncoder encoder;
  
  // WPILib PID Controller
  private final PIDController pidController;

  // Current position and setpoint
  private double setpoint = 0.0;
  
  /** Creates a new CoralArmSubsystem. */
  public CoralArmSubsystem() {
    // Initialize motor controller
    armMotor = new SparkFlex(CoralArmConstants.kCoralArmMotorCanId, MotorType.kBrushless);
    armBore = new SparkMax(Constants.CoralArmConstants.armBoreCanId, MotorType.kBrushless);

    // Get the encoder from the SparkFlex
    encoder = armBore.getAbsoluteEncoder();
    
    // Create and configure WPILib PID controller
    pidController = new PIDController(CoralArmConstants.kP, CoralArmConstants.kI, CoralArmConstants.kD);
    pidController.setTolerance(0.05); // Set tolerance to 0.05 rotations

    SparkFlexConfig config = new SparkFlexConfig();
    config.inverted(CoralArmConstants.isInverted);
    armMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    setpoint = getAngleDegrees();
    setPosition(setpoint);

  }
  
  /**
   * Gets the current position of the arm in degrees
   * @return Current position in degrees
   */
  public double getAngleDegrees() {
    return encoder.getPosition();
  }

  /**
   * Checks if the given angle would enter the forbidden zone
   * @param angleDegrees The angle to check in degrees
   * @return true if the angle is in the forbidden zone, false otherwise
   */
  private boolean isInForbiddenZone(double angleDegrees) {
    return angleDegrees <= CoralArmConstants.FORBIDDEN_ZONE_START && angleDegrees >= CoralArmConstants.FORBIDDEN_ZONE_END;
  }

  /**
   * Checks if the current movement direction would drive into the forbidden zone
   * @param speed The commanded speed (-1.0 to 1.0)
   * @return true if the movement would drive into the forbidden zone, false otherwise
   */
  private boolean isMovingIntoForbiddenZone(double speed) {
    double currentAngle = getAngleDegrees();
    
    // Already in forbidden zone - only allow movement that takes us out
    if (isInForbiddenZone(currentAngle)) {
      // If we're in the first half of the forbidden zone, only allow negative speed (move right)
      if (currentAngle < (CoralArmConstants.FORBIDDEN_ZONE_START + CoralArmConstants.FORBIDDEN_ZONE_END) / 2) {
        return speed > 0;  // Prevent going further left
      } else {
        return speed < 0;  // Prevent going further right
      }
    }

    return false;
  }

  /**
   * Checks if the arm is at the target position
   * @return True if at setpoint
   */
  public boolean atSetpoint() {
    return pidController.atSetpoint();
  }

  // Control mode enum
  public enum ControlMode {
    OPEN_LOOP,
    CLOSED_LOOP
  }

  // Current control mode
  private ControlMode currentMode = ControlMode.CLOSED_LOOP;
  // Last commanded speed for open loop
  private double lastCommandedSpeed = 0.0;

  /**
   * Sets the speed of the arm motor directly (open loop control)
   * This assumes that positive direction is to the left
   * @param speed The speed to set (-1.0 to 1.0)
   */
  public void setSpeed(double speed) {
    lastCommandedSpeed = speed;
    currentMode = ControlMode.OPEN_LOOP;
    armMotor.set(speed);
  }

  /**
   * Sets the target position using PID control (closed loop control)
   * @param position The target position in rotations
   */
  public void setPosition(double position) {
    this.setpoint = position;
    pidController.setSetpoint(position);
    currentMode = ControlMode.CLOSED_LOOP;
  }

  /**
   * Get the current control mode
   * @return Current control mode (OPEN_LOOP or CLOSED_LOOP)
   */
  public ControlMode getControlMode() {
    return currentMode;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double currentAngle = getAngleDegrees();
    double pidOutput = 0.0;
    boolean limitTriggered = false;
    
    // Apply control based on current mode
    if (currentMode == ControlMode.CLOSED_LOOP) {
      // Calculate PID output
      pidOutput = pidController.calculate(getAngleDegrees());
      
      // Apply software limits for closed loop control
      if (isMovingIntoForbiddenZone(pidOutput)) {
        pidOutput = 0;
        limitTriggered = true;
      }
      
      // Apply output to motor
      armMotor.set(pidOutput);
    } else {
      // In open loop mode, nothing needs to be done here
      // The speed was already set in setSpeed() method
      // We just need to check if the limit was triggered for telemetry
      limitTriggered = isMovingIntoForbiddenZone(lastCommandedSpeed);
    }
    
    // Send telemetry to SmartDashboard
    SmartDashboard.putNumber("Arm Position", getAngleDegrees());
    SmartDashboard.putNumber("Arm Setpoint", setpoint);
    SmartDashboard.putBoolean("Arm At Setpoint", atSetpoint());
    SmartDashboard.putString("Arm Control Mode", currentMode.toString());
    SmartDashboard.putBoolean("Arm In Forbidden Zone", isInForbiddenZone(currentAngle));
    SmartDashboard.putBoolean("Arm Limit Triggered", limitTriggered);
    
    if (currentMode == ControlMode.CLOSED_LOOP) {
      SmartDashboard.putNumber("Arm PID Output", pidController.calculate(getAngleDegrees()));
    } else {
      SmartDashboard.putNumber("Arm Open Loop Speed", lastCommandedSpeed);
    }
  }
}