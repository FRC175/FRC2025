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
import edu.wpi.first.wpilibj2.command.button.Trigger;
 
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.manipulatorSetpoint;

import frc.robot.subsystems.*;
import frc.robot.subsystems.Drive.Drive;
import frc.robot.commands.manipulator.Intake;

import frc.robot.commands.manipulator.Discharge;
import frc.robot.commands.Swerve;


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
  
  private final XboxController driverController/* , operatorController*/;
  private final XboxController operatorController;
  private final SendableChooser<Command> autoChooser;
 /// private final Shuckleboard shuffleboard;
  private final Cage cage;
  private final Elevator elevator;
  private final Manipulator manipulator;

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
    

    driverController = new  XboxController(ControllerConstants.DRIVER_CONTROLLER_PORT);
    operatorController = new XboxController(ControllerConstants.OPERATOR_CONTROLLER_PORT);
    
    autoChooser = new SendableChooser<>();

    SmartDashboard.putData("Auto Chooser", autoChooser);
     

    // Configure the default commands
    configureDefaultCommands();

    // Configure the button bindings
    configureButtonBindings();

    // Configure auto mode
    configureAutoChooser();
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
   
   
    //manipulator.setDefaultCommand (new setManipWorking(manipulator, .05, 0.3));

   elevator.setDefaultCommand(new InstantCommand(() -> {}, elevator));
   
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


    new Trigger(() -> operatorController.getBButtonPressed())
    .onTrue(new InstantCommand(() -> {manipulator.setGoalSetpoint(manipulatorSetpoint.ALGAE);;}));
  
     new Trigger(() -> operatorController.getAButtonPressed())
    .onTrue(new InstantCommand(() -> {manipulator.setGoalSetpoint(manipulatorSetpoint.CORAL);;}));
   
     new Trigger(() -> operatorController.getXButtonPressed())
    .onTrue(new InstantCommand(() -> {manipulator.setGoalSetpoint(manipulatorSetpoint.INTAKING);;}));
   
    
    
    // B button ) triggers the cage pneumatics
    
    // new Trigger(() -> operatorController.getBButtonPressed())
    // .onTrue( new SetElevatorPosition(.01, 0.01, 6, elevatorSetpoint.GROUND));
    // // dDpad Down ) sends the elevator to Lowest position (ground))
    // new Trigger(() -> operatorController.getRawButtonPressed(14))
    // .onTrue( new SetElevatorPosition(.01, 0.01, 6, elevatorSetpoint.L1));
    // new Trigger(() -> operatorController.getRawButtonPressed(10))
    // .onTrue( new SetElevatorPosition(.01, 0.01, 6, elevatorSetpoint.L2));
    // new Trigger(() -> operatorController.getRawButtonPressed(6))
    // .onTrue( new SetElevatorPosition(.01, 0.01, 6, elevatorSetpoint.L3));
    // new Trigger(() -> operatorController.getRawButtonPressed(2))
    // .onTrue( new SetElevatorPosition(.01, 0.01, 6, elevatorSetpoint.L4));
   
    // dDpad up) sends the elevator to L4)

    new Trigger(() -> operatorController.getLeftBumperButton())
    .onTrue(
      new Intake(-.1, -0.03)
    )
    .onFalse(new InstantCommand(() -> {
      manipulator.setIntakeOpenLoop(0); }, manipulator));
      //tweak

      new Trigger(() -> operatorController.getRightBumperButton())
      .onTrue(new Discharge(.03))
      .onFalse(new InstantCommand(() -> {
        manipulator.setIntakeOpenLoop(0); }, manipulator));

    // new Trigger(() -> operatorController.getRawButtonPressed(1))
    // .onTrue();

        // button 13 is a spare

  //   new Trigger(() -> operatorController.getRawButtonPressed(5))
  //   .onTrue(new RunCommand(() -> cage.collapseFunnel(), cage));

  //   new Trigger(() -> operatorController.getRawButtonPressed(4))
  //   .whileTrue(new ParallelCommandGroup(new InstantCommand(() -> {
  //     manipulator.manual = true;}), new InstantCommand(() -> {
  //       manipulator.cc = false;})
  //     ));

  //     new Trigger(() -> operatorController.getRawButtonPressed(8))
  //   .whileTrue(new ParallelCommandGroup(new InstantCommand(() -> {
  //     manipulator.manual = true;}), new InstantCommand(() -> {
  //       manipulator.cc = true;})
  //     ));
   
  //     //change to new buttons
  //     new Trigger(() -> operatorController.getRawButton(7))
  //   .onTrue(new SetElevatorPositionManual(true, 0.1))
  //   .onFalse(new InstantCommand(() -> elevator.setOpenLoop(0), elevator));

  //     new Trigger(() -> operatorController.getRawButton(11))
  //     .onTrue(new SetElevatorPositionManual(false, -0.1))
  //     .onFalse(new InstantCommand(() -> elevator.setOpenLoop(0)));

  //     new Trigger(() -> driverController.getLeftBumperButton())
  //     .onTrue(new InstantCommand(() -> drive.resetGyro(0), drive));
  // 
  }




  private void configureAutoChooser() {
    
  }

  //test

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

