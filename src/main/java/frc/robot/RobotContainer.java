package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.CoralScoringMechanismConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Commands.AlgaeCommands.AlgaeIntakeBiDirectionalCommand;
import frc.robot.Commands.AlgaeCommands.AlgaeWristOpenLoopCommand;
import frc.robot.Commands.ClimbCommands.ClimbBiDirectionalCommand;
import frc.robot.Commands.CoralCommands.ArmMechGoToPos_CMD;
import frc.robot.Commands.CoralCommands.CoralArmOpenLoopCommand;
import frc.robot.Commands.CoralCommands.CoralInsertBiDirectionalCommand;
import frc.robot.Commands.CoralCommands.CoralWristOpenLoopCommand;
import frc.robot.Commands.CoralCommands.InAndOutCoral_CMD;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.subsystems.AlgaeWristSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.CoralArmSubsystem;
import frc.robot.subsystems.CoralScoringMechanism;
import frc.robot.subsystems.CoralWristSubsystem;
import frc.robot.subsystems.DriveSubsystemOld;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.List;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystemOld m_robotDrive = new DriveSubsystemOld();
  private final CoralWristSubsystem m_coralWristSubsystem = new CoralWristSubsystem();
  private final CoralArmSubsystem m_coralArmSubsystem = new CoralArmSubsystem();
  private final CoralScoringMechanism m_coralInsertMechanism = new CoralScoringMechanism();
  private final AlgaeWristSubsystem m_algaeWristSubsystem = new AlgaeWristSubsystem();
  private final AlgaeIntakeSubsystem m_algaeIntakeSubsystem = new AlgaeIntakeSubsystem();
  private final ClimbSubsystem m_climbSubsystem = new ClimbSubsystem();
  
//   SendableChooser<Command> autonChooser;

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  private final CommandXboxController operatorController = new CommandXboxController(
      OperatorConstants.OPERATOR_CONTROLLER_PORT);
    private final CommandXboxController backUpOperator = new CommandXboxController(
      OperatorConstants.BACK_UP_OPERATOR_CONTROLLER_PORT);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    
    //------------------------ AUTO CODE ---------------------
    //Add named commands here
    //NamedCommands.registerCommand("DriveToAprilTag", new DriveToAprilTag(drivetrain, stateMachine));
    // NamedCommands.registerCommand("IntakPos", 
    //     new ArmMechGoToPos_CMD(m_coralArmSubsystem, m_coralWristSubsystem, Constants.CoralArmConstants.ARM_INTAKE_ANGLE, Constants.CoralWristConstants.WRIST_INTAKE_ANGLE));
    // NamedCommands.registerCommand("L2Pos", 
    //     new ArmMechGoToPos_CMD(m_coralArmSubsystem, m_coralWristSubsystem, Constants.CoralArmConstants.ARM_L2_ANGLE, Constants.CoralWristConstants.WRIST_L2_ANGLE));
    // NamedCommands.registerCommand("L3Pos", 
    //     new ArmMechGoToPos_CMD(m_coralArmSubsystem, m_coralWristSubsystem, Constants.CoralArmConstants.ARM_L3_ANGLE, Constants.CoralWristConstants.WRIST_L3_ANGLE));
    // NamedCommands.registerCommand("IntakCoral", new InAndOutCoral_CMD(m_coralInsertMechanism, false));
    // NamedCommands.registerCommand("ShootCoral", new InAndOutCoral_CMD(m_coralInsertMechanism, true));


    // autonChooser = AutoBuilder.buildAutoChooser();
    // SmartDashboard.putData("Auto", autonChooser);
    // Configure the button bindings
    
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true),
            m_robotDrive));
    
     // Default command for Coral Wrist control with operator right stick
     m_coralWristSubsystem.setDefaultCommand(
      new CoralWristOpenLoopCommand(
          m_coralWristSubsystem,
          () -> -MathUtil.applyDeadband(operatorController.getRightY(), 0.1) * 0.5
      )
  );
  
  // Default command for Coral Arm with operator left stick
  m_coralArmSubsystem.setDefaultCommand(
      new CoralArmOpenLoopCommand(
          m_coralArmSubsystem,
          () -> -MathUtil.applyDeadband(operatorController.getLeftY(), 0.1) * 0.5
      )
  );
  
  // Default command for Algae Wrist with backup operator right stick
  m_algaeWristSubsystem.setDefaultCommand(
      new AlgaeWristOpenLoopCommand(
          m_algaeWristSubsystem,
          () -> -MathUtil.applyDeadband(backUpOperator.getRightY(), 0.1) * 0.5
      )
  );
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    // Keep existing button bindings
    new JoystickButton(m_driverController, Button.kR1.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));
            
    // CORAL CONTROLS (OPERATOR CONTROLLER)
    
    // Coral Insert Mechanism Control
    operatorController.a().whileTrue(
        new CoralInsertBiDirectionalCommand(m_coralInsertMechanism, CoralScoringMechanismConstants.kInsertSpeed)
    );
    
    operatorController.b().whileTrue(
        new CoralInsertBiDirectionalCommand(m_coralInsertMechanism, CoralScoringMechanismConstants.kEjectSpeed)
    );

    operatorController.rightTrigger(0.2).whileTrue(new InAndOutCoral_CMD(m_coralInsertMechanism, false));
    operatorController.leftTrigger(0.2).whileTrue(new InAndOutCoral_CMD(m_coralInsertMechanism, true));

    //intake coral position
    operatorController.povDown().onTrue(
        new ArmMechGoToPos_CMD(m_coralArmSubsystem, m_coralWristSubsystem, Constants.CoralArmConstants.ARM_INTAKE_ANGLE, Constants.CoralWristConstants.WRIST_INTAKE_ANGLE));
    //score l2 position
    operatorController.povRight().onTrue(
        new ArmMechGoToPos_CMD(m_coralArmSubsystem, m_coralWristSubsystem, Constants.CoralArmConstants.ARM_L2_ANGLE, Constants.CoralWristConstants.WRIST_L2_ANGLE));
    //score l3 position
    operatorController.povUp().onTrue(
        new ArmMechGoToPos_CMD(m_coralArmSubsystem, m_coralWristSubsystem, Constants.CoralArmConstants.ARM_L3_ANGLE, Constants.CoralWristConstants.WRIST_L3_ANGLE));

    
    // // Manual override for Coral Wrist
    // operatorController.leftBumper().whileTrue(
    //     new CoralWristOpenLoopCommand(m_coralWristSubsystem, () -> 0.3)
    // );
    
    // operatorController.rightBumper().whileTrue(
    //     new CoralWristOpenLoopCommand(m_coralWristSubsystem, () -> -0.3)
    // );
    
    // // Manual override for Coral Arm
    // operatorController.povUp().whileTrue(
    //     new CoralArmOpenLoopCommand(m_coralArmSubsystem, () -> 0.3)
    // );
    
    // operatorController.povDown().whileTrue(
    //     new CoralArmOpenLoopCommand(m_coralArmSubsystem, () -> -0.3)
    // );
    
    // ALGAE CONTROLS (BACKUP OPERATOR CONTROLLER)
    
    // Algae Intake Control
    backUpOperator.leftTrigger(0.1).whileTrue(
        new AlgaeIntakeBiDirectionalCommand(
            m_algaeIntakeSubsystem,
            () -> backUpOperator.getLeftTriggerAxis()
        )
    );
    
    backUpOperator.rightTrigger(0.1).whileTrue(
        new AlgaeIntakeBiDirectionalCommand(
            m_algaeIntakeSubsystem,
            () -> -backUpOperator.getRightTriggerAxis()
        )
    );

    
    // // Manual override for Algae Wrist
    // backUpOperator.leftBumper().whileTrue(
    //     new AlgaeWristOpenLoopCommand(m_algaeWristSubsystem, () -> 0.3)
    // );
    
    // backUpOperator.rightBumper().whileTrue(
    //     new AlgaeWristOpenLoopCommand(m_algaeWristSubsystem, () -> -0.3)
    // );
    
    // CLIMB CONTROLS (DRIVER CONTROLLER)
    
    // Climb Control with driver triggers
    new Trigger(() -> Math.abs(m_driverController.getRightTriggerAxis()) > 0.1)
        .whileTrue(new ClimbBiDirectionalCommand(
            m_climbSubsystem,
            () -> m_driverController.getRightTriggerAxis() * 0.7 // Scale to 70% max speed
        ));
    
    new Trigger(() -> Math.abs(m_driverController.getLeftTriggerAxis()) > 0.1)
        .whileTrue(new ClimbBiDirectionalCommand(
            m_climbSubsystem,
            () -> -m_driverController.getLeftTriggerAxis() * 0.7 // Scale to 70% max speed
        ));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
//   public Command getAutonomousCommand() {
//     return autonChooser.getSelected();
//   }
}