package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeIntakeConstants;
import frc.robot.Constants.AlgaeWristConstants;

public class AlgaeIntakeSubsystem extends SubsystemBase {
  // Motors for the intake
  private final SparkMax leftIntakeMotor;
  private final SparkMax rightIntakeMotor;
  
  // Current commanded speed
  private double currentSpeed = 0.0;

  /** Creates a new AlgaeIntakeSubsystem with dual motors. */
  public AlgaeIntakeSubsystem() {
    // Initialize the motors
    leftIntakeMotor = new SparkMax(AlgaeIntakeConstants.kLeftIntakeMotorCanId, MotorType.kBrushless);
    rightIntakeMotor = new SparkMax(AlgaeIntakeConstants.kRightIntakeMotorCanId, MotorType.kBrushless);
    
    // Configure left motor
    SparkMaxConfig leftConfig = new SparkMaxConfig();
    leftConfig.inverted(AlgaeIntakeConstants.kLeftIntakeMotorInverted);
    
    // Configure right motor - inverted opposite of left by default
    SparkMaxConfig rightConfig = new SparkMaxConfig();
    rightConfig.inverted(AlgaeIntakeConstants.kLeftIntakeMotorInverted);
    
    // Apply configurations
    leftIntakeMotor.configure(leftConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    rightIntakeMotor.configure(rightConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  /**
   * Set the speed of both intake motors
   * Positive speed means intaking (both motors spin inward)
   * @param speed The speed to set (-1.0 to 1.0)
   */
  public void setSpeed(double speed) {
    currentSpeed = speed;
    leftIntakeMotor.set(speed);
    rightIntakeMotor.set(speed);
  }

  /**
   * Stops both intake motors
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
    SmartDashboard.putNumber("Left Intake Current", leftIntakeMotor.getOutputCurrent());
    SmartDashboard.putNumber("Right Intake Current", rightIntakeMotor.getOutputCurrent());
  }
}