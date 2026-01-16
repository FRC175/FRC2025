// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.concurrent.CyclicBarrier;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
 

import frc.robot.utils.Controller;
import frc.robot.utils.Utils;


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
; 
  
  private final XboxController driverController/* , operatorController*/;

  private final SendableChooser<Command> autoChooser;
 /// private final Shuckleboard shuffleboard;


  private static RobotContainer instance;
  private static Limelight limelight;


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    //this.shuffleboard = Shuckleboard.getInstance();
 
   
    driverController = new  XboxController(0);
   
    autoChooser = new SendableChooser<>();
    limelight = new Limelight();

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

    // drive.setDefaultCommand(new Swerve(driverController, drive));

    // cage.setDefaultCommand(new RunCommand(() -> {
    //   cage.enableCompressor();
    // } , cage));
    
    //manipulator.setDefaultCommand (new setManipWorking(manipulator, .05, 0.3));

 
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  
    // B button ) triggers the cage pneumatics
    
    // new Trigger(() -> operatorController.getBButtonPressed())
    // .onTrue( new SetElevatorPosition(.01, 0.01, 6, elevatorSetpoint.GROUND));
   
   // dDpad up) sends the elevator to L4)

   
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

