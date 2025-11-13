// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import java.io.File;
import java.util.ArrayList;
import java.util.HashMap;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
 
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.ElevatorSetpoint;
import frc.robot.Constants.intakePoints;
import frc.robot.Constants.manipulatorSetpoint;

import frc.robot.subsystems.*;
import swervelib.SwerveInputStream;
import frc.robot.commands.Elevator.ControlElevator;
import frc.robot.commands.Elevator.SetElevatorPosition;




/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  // private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  
 
  //private final GenericHID operatorController;
  private final SendableChooser<Command> autoChooser;
 /// private final Shuckleboard shuffleboard;
  private final Cage cage;
  private final Elevator elevator;
  private final Manipulator manipulator;
  private final Intake intake;
  
//private final SwerveSubsystem drive = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  
  private static RobotContainer instance;

  private final Field2d field;

  private PathPlannerLogging pLogging;

  
   

  final CommandXboxController driverController = new  CommandXboxController(ControllerConstants.DRIVER_CONTROLLER_PORT);
    //operatorController = new GenericHID(ControllerConstants.OPERATOR_CONTROLLER_PORT);
 
  final CommandXboxController operatorController = new  CommandXboxController(ControllerConstants.OPERATOR_CONTROLLER_PORT);
    //operatorController = new GenericHID(ControllerConstants.OPERATOR_CONTROLLER_PORT);
 
    
  private final SwerveSubsytem drivebase = new SwerveSubsytem(new File(Filesystem.getDeployDirectory(),"swerve"));



  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
  () -> driverController.getLeftY() * -1, () -> driverController.getLeftX() * -1)
    .withControllerRotationAxis(driverController::getRightX)
    .deadband(ControllerConstants.DEADBAND)
    .scaleTranslation(0.8)
    .allianceRelativeControl(true);


  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
  () -> -driverController.getLeftY(),
  () -> -driverController.getLeftX())
  .withControllerRotationAxis(() -> driverController.getRawAxis(
  2))
  .deadband(ControllerConstants.DEADBAND)
  .scaleTranslation(0.8)
  .allianceRelativeControl(true);


  /**
  * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
  */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverController::getRightX, driverController::getRightY)
  .headingWhile(true);

  SwerveInputStream driveDirectAngleKeyboard = driveAngularVelocityKeyboard.copy().withControllerHeadingAxis(() ->
      Math.sin( driverController.getRawAxis(2) *Math.PI) *(Math.PI *2),() ->
        Math.cos(driverController.getRawAxis( 2) * Math.PI) * (Math.PI * 2))
          .headingWhile(true)
          .translationHeadingOffset(true)
          .translationHeadingOffset(Rotation2d.fromDegrees(0));




  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    //this.shuffleboard = Shuckleboard.getInstance();
    this.cage = Cage.getInstance();
    this.elevator = Elevator.getInstance();
    this.manipulator = Manipulator.getInstance();
  
    this.intake = Intake.getInstance();
    this.field = new Field2d();

    pLogging = new PathPlannerLogging();

    
  



    autoChooser = new SendableChooser<>();

     /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
 


    
    // Configure the default commands
    configureDefaultCommands();

    // Configure the button bindings
    configureButtonBindings();

    // Configure auto mode
    configureAutoChooser();

    // Configure YAGSL swerve commands
    configureSwerveBindings();
      
    
       
            
    
           
      
        
       
        
    
            // Logging callback for the active path, this is sent as a list of poses
            
      }
    
      private void configureSwerveBindings() {
        
      Command driveFieldOrientedDirectAngle      = drivebase.driveFieldOriented(driveDirectAngle);
      Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
      Command driveFieldOrientedDirectAngleKeyboard      = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
      Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);

        if (RobotBase.isSimulation())
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    } else
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    if (Robot.isSimulation())
    {
      Pose2d target = new Pose2d(new Translation2d(1, 4), Rotation2d.fromDegrees(90));
      //drivebase.getSwerveDrive().field.getObject("targetPose").setPose(target);
      driveDirectAngleKeyboard.driveToPose(() -> 
        target, new ProfiledPIDController(5, 0, 0, new Constraints(5, 2)), new ProfiledPIDController(5, 0, 0, new Constraints(Units.degreesToRadians(360), Units.degreesToRadians(180))
      ));
      driverController.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      driverController.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());
      driverController.button(2).whileTrue(Commands.runEnd(() -> driveDirectAngleKeyboard.driveToPoseEnabled(true), () -> driveDirectAngleKeyboard.driveToPoseEnabled(false)));

//      driverController.b().whileTrue(
//          drivebase.driveToPose(
//              new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
//                              );

    }
    if (DriverStation.isTest())
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

      driverController.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverController.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      driverController.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverController.back().whileTrue(drivebase.centerModulesCommand());
      driverController.leftBumper().onTrue(Commands.none());
      driverController.rightBumper().onTrue(Commands.none());
    } else
    {
      driverController.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverController.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      driverController.start().whileTrue(Commands.none());
      driverController.back().whileTrue(Commands.none());
      driverController.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverController.rightBumper().onTrue(Commands.none());
    }
 
      }
    
      public static RobotContainer getInstance() {
    if (instance == null) {
        instance = new RobotContainer();
    }

    return instance;
  }

  
  


  private void configureDefaultCommands() {

    

    elevator.setDefaultCommand(new ControlElevator(0.85, 0.45, 915));


  }


  //  * Use this method to define your button->command mappings. Buttons can be created by
  //  * instantiating a {@link GenericHID} or one of its subclasses ({@link
  //  * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
  //  * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
  //  */
  private void configureButtonBindings() {


        new Trigger(() -> operatorController.getRightTriggerAxis() > .1) 
          .whileTrue(new InstantCommand(() -> {
            intake.setState(intakePoints.HOLD_ALGAE);}))

          .onFalse(new InstantCommand(() -> {
            intake.setState(intakePoints.OFF);
          }));
        

        new Trigger(() -> operatorController.getLeftTriggerAxis() > .1)
        .onTrue(new InstantCommand(() -> { 
          manipulator.setGoalPoint(manipulatorSetpoint.L4CORAL.getSetpoint());
        }));
        

  }

  private void configureAutoChooser() {

   
    SmartDashboard.putData(autoChooser);
  }
//nr[p,///.]
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    // An ExampleCommand will run in autonomous
    
    return autoChooser.getSelected();
    //return new B2L4(drive, intake, manipulator, elevator);
  }
}

