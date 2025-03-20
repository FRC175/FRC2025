// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
 
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.ElevatorSetpoint;
import frc.robot.Constants.intakePoints;
import frc.robot.Constants.manipulatorSetpoint;

import frc.robot.subsystems.*;
import frc.robot.subsystems.Drive.Drive;
import frc.robot.commands.runIntake;
import frc.robot.commands.setManipWorking;
import frc.robot.commands.Auto.leave;
import frc.robot.commands.Drive.Swerve;
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
  
  private final XboxController driverController, operatorController/* , operatorController*/;
  //private final GenericHID operatorController;
  private final SendableChooser<Command> autoChooser;
 /// private final Shuckleboard shuffleboard;
  private final Cage cage;
  private final Elevator elevator;
  private final Manipulator manipulator;
  private final Intake intake;
//private final SwerveSubsystem drive = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  
  private static RobotContainer instance;
  private final Drive drive;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    //this.shuffleboard = Shuckleboard.getInstance();
    this.cage = Cage.getInstance();
    this.elevator = Elevator.getInstance();
    this.manipulator = Manipulator.getInstance();
    this.drive = Drive.getInstance();
    this.intake = Intake.getInstance();

    driverController = new  XboxController(ControllerConstants.DRIVER_CONTROLLER_PORT);
    //operatorController = new GenericHID(ControllerConstants.OPERATOR_CONTROLLER_PORT);
    operatorController = new XboxController(ControllerConstants.OPERATOR_CONTROLLER_PORT);
    
    autoChooser = new SendableChooser<>();

    

    // Configure the default commands
    configureDefaultCommands();

    // Configure the button bindings
    configureButtonBindings();

    // Configure auto mode
    configureAutoChooser();

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  public static RobotContainer getInstance() {
    if (instance == null) {
        instance = new RobotContainer();
    }

    return instance;
  }

  // public void registerCommandsAuto() {
  //     //NamedCommands.registerCommand("SwerveToTag", new SwerveToTag(drive));
  //     NamedCommands.registerCommand("Intake", new Intake(.03, .01));
  //     NamedCommands.registerCommand("Discharge", new Discharge(.03));
  // }

  private void configureDefaultCommands() {

    drive.setDefaultCommand(new Swerve(driverController, drive));

    // cage.setDefaultCommand(new RunCommand(() -> {
    //   cage.enableCompressor();
    // } , cage));
    // drive.setDefaultCommand(drive.driveRelativeCommand(() ->
    //  MathUtil.applyDeadband(-1*driverController.getLeftX(), Constants.DriveConstants.driveDeadbandY, Constants.DriveConstants.MAXIMUMSPEED),
    // () -> MathUtil.applyDeadband(driverController.getLeftY(), Constants.DriveConstants.driveDeadbandX, Constants.DriveConstants.MAXIMUMSPEED),
    // () -> MathUtil.applyDeadband(driverController.getRightX(), Constants.DriveConstants.driveDeadbandX, Constants.DriveConstants.MAXIMUMSPEED)));
   
   
    manipulator.setDefaultCommand ( new setManipWorking(manipulator, .28, 0.5, 0.5));
    intake.setDefaultCommand( new runIntake(.5));
    elevator.setDefaultCommand(new ControlElevator(0.85, 0.45, 915));


  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new Trigger (() -> driverController.getLeftBumperButton())
    .onTrue(new InstantCommand(() -> drive.resetGyro(0), drive));

    new Trigger (() -> driverController.getRightBumperButton())
    .onTrue(new ParallelCommandGroup(new InstantCommand(() -> drive.setPrevYaw(drive.getRawYaw())), new InstantCommand(() -> drive.resetGyro(0))))
    .onFalse(new InstantCommand(() -> drive.resetGyro(drive.getPrevYaw())));

    // new Trigger(() -> operatorController.getRawButton(9))
    // .onTrue( new InstantCommand(() -> {
    //   manipulator.setGoalPoint(manipulatorSetpoint.CORALTRAVEL.getSetpoint());}));
    
    new Trigger(() -> operatorController.getLeftBumperButtonPressed())
      .onTrue( new InstantCommand(() -> {
        manipulator.setGoalPoint(manipulatorSetpoint.CORALTRAVEL.getSetpoint());}));
    
    // new Trigger(() -> operatorController.getRawButton(13))
    // .onTrue( new InstantCommand(() -> {
    //     manipulator.setGoalPoint(manipulatorSetpoint.CORALIN.getSetpoint());}));
  

    
    //  new Trigger(() -> operatorController.getRawButtonPressed(14))
    //   .onTrue(new SetElevatorPosition(manipulator, elevator, ElevatorSetpoint.L1));

     new Trigger(() -> operatorController.getPOV() == 0)
      .onTrue(new SetElevatorPosition(manipulator, elevator, ElevatorSetpoint.L1));
    
    // new Trigger(() -> operatorController.getRawButtonPressed(10))
    //   .onTrue(new SetElevatorPosition(manipulator, elevator, ElevatorSetpoint.L2));
    
     new Trigger(() -> operatorController.getPOV() == 90)
      .onTrue(new SetElevatorPosition(manipulator, elevator, ElevatorSetpoint.L2));
    
    
    // new Trigger(() -> operatorController.getRawButtonPressed(6))
    //   .onTrue(new SetElevatorPosition(manipulator, elevator, ElevatorSetpoint.L3));

    new Trigger(() -> operatorController.getPOV() == 180)
      .onTrue(new SetElevatorPosition(manipulator, elevator, ElevatorSetpoint.L3));
    
    
    // new Trigger(() -> operatorController.getRawButtonPressed(2))
    //   .onTrue(new SetElevatorPosition(manipulator, elevator, ElevatorSetpoint.L4));

     new Trigger(() -> operatorController.getPOV() == 270)
      .onTrue(new SetElevatorPosition(manipulator, elevator, ElevatorSetpoint.L4));
    


    // new Trigger(() -> operatorController.getRawButton(16))
    //   .onTrue(new InstantCommand(() -> intake.setState(intakePoints.INTAKE_CORAL)))
    //   .onFalse(new InstantCommand(() -> intake.setState(intakePoints.OFF)));

      new Trigger(() -> operatorController.getYButton())
      .onTrue(new InstantCommand(() -> intake.setState(intakePoints.INTAKE_CORAL)))
      .onFalse(new InstantCommand(() -> intake.setState(intakePoints.OFF)));


    // new Trigger(() -> operatorController.getRawButton(15))
    //   .onTrue(new InstantCommand(() ->  intake.setState(intakePoints.DISCHARGE_CORAL)))
    //   .onFalse(new InstantCommand(() -> intake.setState(intakePoints.OFF)));

      new Trigger(() -> operatorController.getBButton())
      .onTrue(new InstantCommand(() -> intake.setState(intakePoints.DISCHARGE_CORAL)))
      .onFalse(new InstantCommand(() -> intake.setState(intakePoints.OFF)));


    //  new Trigger(() -> operatorController.getRawButtonPressed(5))
    // .onTrue(new InstantCommand(() -> {
    //   manipulator.setGoalPoint(manipulatorSetpoint.ALGAEIN.getSetpoint());}));

      new Trigger(() -> operatorController.getRightBumperButtonPressed())
      .onTrue(new InstantCommand(() -> {
        manipulator.setGoalPoint(manipulatorSetpoint.ALGAEIN.getSetpoint());}));
  

   
   
      //change to new buttons
      // new Trigger(() -> operatorController.getRawButton(7))
      //   .onTrue(new InstantCommand(() -> { 
      //     elevator.setGoalPoint(elevator.getGoalSetpoint() + 150.0); 
      //   }));

      // new Trigger(() -> operatorController.getRawButton(11))
      //   .onTrue(new InstantCommand(() -> { 
      //     elevator.setGoalPoint(elevator.getGoalSetpoint() - 150.0); 
      //   }));

      // new Trigger(() -> operatorController.getRawButton(4))
      //   .onTrue(new InstantCommand(() -> { 
      //     manipulator.setGoalPoint(manipulator.getGoalSetpoint() + .05); 
      //   }));

      // new Trigger(() -> operatorController.getRawButton(8))
      //   .onTrue(new InstantCommand(() -> { 
      //     manipulator.setGoalPoint(manipulator.getGoalSetpoint() - .05); 
      //   }));

      //   new Trigger(() -> operatorController.getRawButton(8))
      //   .onTrue(new InstantCommand(() -> { 
      //     manipulator.setGoalPoint(manipulator.getGoalSetpoint() - .05); 
      //   }));

        new Trigger(() -> operatorController.getXButton())
        .onTrue(new InstantCommand(() -> { 
          intake.setState(intakePoints.INTAKE_ALGAE);}))
        
        .onFalse (new InstantCommand(() -> { 
          intake.setState(intakePoints.OFF);})
        );

        new Trigger(() -> operatorController.getLeftTriggerAxis() > .1)
        .onTrue(new InstantCommand(() -> { 
          manipulator.setGoalPoint(manipulatorSetpoint.L4CORAL.getSetpoint());
        }));
        

        // new Trigger(() -> operatorController.getRawButton(1))
        // .onTrue(new InstantCommand(() -> { 
        //   intake.setState(intakePoints.DISCHARGEALGAE);
        // }))
        // .onFalse (new InstantCommand(() -> { 
        //   intake.setState(intakePoints.OFF);})
        // );
  }

  private void configureAutoChooser() {
    autoChooser.setDefaultOption("Nothing", new ParallelCommandGroup(new WaitCommand(0), new InstantCommand(() -> drive.resetGyro(0))));
    autoChooser.addOption("leave", new leave(drive));
   
    SmartDashboard.putData(autoChooser);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return autoChooser.getSelected();
  }
}

