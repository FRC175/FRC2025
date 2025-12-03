// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import java.util.ArrayList;
import java.util.HashMap;

// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.auto.NamedCommands;
// import com.pathplanner.lib.config.PIDConstants;
// import com.pathplanner.lib.path.PathConstraints;
// import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
// import com.pathplanner.lib.util.PathPlannerLogging;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
 
// import frc.robot.Constants.ControllerConstants;
// import frc.robot.Constants.ElevatorConstants.ElevatorSetpoint;
// import frc.robot.Constants.ElevatorConstants.intakePoints;
// import frc.robot.Constants.ElevatorConstants.manipulatorSetpoint;

import frc.robot.subsystems.*;




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
  
  private final XboxController driverController, operatorController/* , operatorController*/;
 
  private final SendableChooser<Command> autoChooser;

  private final Elevator breadboard;
  private static RobotContainer instance;



  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    //this.shuffleboard = Shuckleboard.getInstance();
  

    

    driverController = new  XboxController(0);
    // //operatorController = new GenericHID(ControllerConstants.OPERATOR_CONTROLLER_PORT);
     operatorController = new XboxController(1);

  breadboard = Elevator.getInstance();



    autoChooser = new SendableChooser<>();

    
    // Configure the default commands
    configureDefaultCommands();

    // Configure the button bindings
    configureButtonBindings();

    // Configure auto mode
    configureAutoChooser();
  

   
        

       
  
    
   
    

        // Logging callback for the active path, this is sent as a list of poses
        
  }

  public static RobotContainer getInstance() {
    if (instance == null) {
        instance = new RobotContainer();
    }

    return instance;
  }

  
  

  // public void registerCommandsAuto() {
  //     //NamedCommands.registerCommand("SwerveToTag", new SwerveToTag(drive));
  //     NamedCommands.registerCommand("Intake", new InstantCommand(() -> intake.setState(intakePoints.INTAKE_CORAL)));
  //     NamedCommands.registerCommand("Discharge", new InstantCommand(() -> intake.setState(intakePoints.DISCHARGE_CORAL)));
  //     NamedCommands.registerCommand("L1", new SetElevatorPosition(manipulator, elevator, ElevatorSetpoint.L1));
  //     NamedCommands.registerCommand("L2", new SetElevatorPosition(manipulator, elevator, ElevatorSetpoint.L2));
  //     NamedCommands.registerCommand("L3", new SetElevatorPosition(manipulator, elevator, ElevatorSetpoint.L3));
  //     NamedCommands.registerCommand("L4", new SetElevatorPosition(manipulator, elevator, ElevatorSetpoint.L4));
  // }

  private void configureDefaultCommands() {



    // cage.setDefaultCommand(new RunCommand(() -> {
    //   cage.enableCompressor();
    // } , cage));
    // drive.setDefaultCommand(drive.driveRelativeCommand(() ->
    //  MathUtil.applyDeadband(-1*driverController.getLeftX(), Constants.DriveConstants.driveDeadbandY, Constants.DriveConstants.MAXIMUMSPEED),
    // () -> MathUtil.applyDeadband(driverController.getLeftY(), Constants.DriveConstants.driveDeadbandX, Constants.DriveConstants.MAXIMUMSPEED),
    // () -> MathUtil.applyDeadband(driverController.getRightX(), Constants.DriveConstants.driveDeadbandX, Constants.DriveConstants.MAXIMUMSPEED)));
   
   


  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new Trigger(() -> driverController.getAButton()) 
      .whileTrue(new InstantCommand(() -> {
        breadboard.setOpenLoopR(1);
      }, breadboard))
      .onFalse(new InstantCommand(() -> {
        breadboard.setOpenLoopR(0);
      }, breadboard));

      new Trigger(() -> driverController.getBButton()) 
      .whileTrue(new InstantCommand(() -> {
        breadboard.setOpenLoopL(1);
      }, breadboard))
      .onFalse(new InstantCommand(() -> {
        breadboard.setOpenLoopL(0);
      }, breadboard));
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

