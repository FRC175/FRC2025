// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.concurrent.CyclicBarrier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
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
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmPosition;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.LiftConstants;

import frc.robot.commands.LED.BlinkAlliance;
import frc.robot.commands.LED.BlinkCycle;
import frc.robot.commands.LED.SetDuring;

import frc.robot.subsystems.LED.LEDColor;
import frc.robot.utils.Controller;
import frc.robot.utils.Utils;

import frc.robot.subsystems.LED;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Drive.SwerveSubsystem;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Limelight;


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

 // private final Intake intake; 
  private final Shooter shooter; 
  private final Limelight limelight;
  private final Lift lift; 
  // private final Arm arm; 
  private final LED led;
  private final Controller driverController/* , operatorController*/;
  private final XboxController operatorController;
  private final SendableChooser<Command> autoChooser;

  private final SwerveSubsystem drive = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  
  PrintWriter writer;
  

  double lockAngle = 0;

  private static RobotContainer instance;



  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
  
    // intake = Intake.getInstance(); 
    shooter = Shooter.getInstance();
    limelight = Limelight.getInstance();
    lift = Lift.getInstance(); 
    // arm = Arm.getInstance();
    led = LED.getInstance();

    driverController = new Controller(new Joystick(ControllerConstants.DRIVER_CONTROLLER_PORT));
    //operatorController = new Joystick(ControllerConstants.OPERATOR_CONTROLLER_PORT);
    operatorController = new XboxController(ControllerConstants.OPERATOR_CONTROLLER_PORT);

    autoChooser = new SendableChooser<>();
    
     

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

  private void configureDefaultCommands() {
    drive.driveCommand(
       () -> MathUtil.applyDeadband(driverController.getLeftX(), Constants.DriveConstants.driveDeadbandX, Constants.DriveConstants.MAXIMUMSPEED),
       () -> MathUtil.applyDeadband(driverController.getLeftY(), Constants.DriveConstants.driveDeadbandX, Constants.DriveConstants.MAXIMUMSPEED),
       () -> driverController.getTwist());

      led.setDefaultCommand(new BlinkAlliance());
  }


  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

  
      // .onFalse(new Swerve(driverController, drive));
     

    // Operator Controller X Button: Reverse Intake 
   
      

    // Operator Controller B Button: Amp 
    // new Trigger(() -> operatorController.getBButton())
    //   .onTrue(new Amp(intake, shooter, 0.4))
    //   .onFalse(new Amp(intake, shooter, 0));
    
    // // Operator Controller B Button: Shoot in Speaker 
    // new Trigger(() -> operatorController.getBButton() )
    // .onTrue(new InstantCommand(() -> {
    // shooter.shooterSetOpenLoop(0.5,0.5);
    // // System.out.println("const");
    // }, shooter))
    // .onFalse(new InstantCommand(() -> {
    //   shooter.shooterSetOpenLoop(0.0, 0.0);
    // }, shooter));

    // Operator Controller Left Trigger: Intake 
    // new Trigger(() -> operatorController.getLeftTriggerAxis() > 0.2)
    // .onTrue(new pickup(intake, false))
    // .whileTrue(new RunCommand(() -> {
    //   if (intake.isNoteHeld()) {
        // System.out.println("present");
        
      //   operatorController.setRumble(RumbleType.kBothRumble, 1.0);
      // } else {
      //   // System.out.println("not present");
      //   operatorController.setRumble(RumbleType.kBothRumble, 0);
      // }}))
    // .onTrue(new RunCommand(() -> {

    //   if (intake.isNotePresent()) {
    //     operatorController.setRumble(RumbleType.kBothRumble, 1.0);
    //     intake.setOpenLoop(0);
    //   } else {
    //     intake.setOpenLoop(0.5);
    //     operatorController.setRumble(RumbleType.kBothRumble, 0);
    //   }
    // }))
    // .onFalse(new InstantCommand(() -> {
    //   intake.setOpenLoop(0);
    //   operatorController.setRumble(RumbleType.kBothRumble, 0);
    // }, intake));

    // new Trigger(() -> intake.isNoteHeld())
    //   .whileTrue(new BlinkCycle(1, 5, new LEDColor[] {LEDColor.ORANGE, DriverStation.getAlliance().get() == Alliance.Red ? LEDColor.RED : LEDColor.BLUE}))
    //   .onFalse(new BlinkAlliance());


    // new Trigger(() -> operatorController.getLeftBumper()) 
    // .onTrue(new RevShooter(shooter, 0.8 * 3600, 0.8 * 2800))
    // .onFalse(new InstantCommand(() -> {
    //   shooter.shooterSetOpenLoop(0, 0);
    // }, shooter));

    // new Trigger(() -> operatorController.getRightTriggerAxis() > 0.5)
    // .onTrue(new RevShooter(shooter, 3500, 3500)) // 51 42
    // .onFalse(new InstantCommand(() -> {
    //   shooter.shooterSetOpenLoop(0, 0);
    // }, shooter));

    // new Trigger(() -> operatorController.getAButton())
    //   .onTrue(new InstantCommand(() -> intake.setOpenLoop(0.4), intake))
    //   .onFalse(new InstantCommand(() -> intake.setOpenLoop(0), intake));

    // // Operator DPad Down: Set Arm Intake Position
    // new Trigger(() -> operatorController.getPOV() == 180)
    //   // .onTrue(new SetArmPosition(arm, 0.2, false, ArmConstants.INTAKE));
    //   .onTrue(new InstantCommand(() -> {
    //     arm.setArmGoalPosition(ArmPosition.INTAKE);
    //   }));

    // new Trigger(() -> Math.abs(operatorController.getRightY()) > 0.5)
    //   .whileTrue(new RunCommand(() -> {
    //     if (operatorController.getRightY() < 0) {
    //         arm.setArmGoalPosition(arm.getArmGoalPosition() - 0.001);
    //     } else if (operatorController.getRightY() > 0) {
    //       arm.setArmGoalPosition(arm.getArmGoalPosition() + 0.001);
    //     }

        
    //   }));
      // new Trigger(() -> operatorController.getLeftStickButton())
      // .onTrue(new BlinkCycle(1, 5, new LEDColor[] {LEDColor.ORANGE, LEDColor.BLACK}))
      // .onFalse(new BlinkAlliance());
      
      

   //  Operator Dpad Up: Set Arm Amp Position 
    // new Trigger(() -> operatorController.getPOV() == 90)
    //   .onTrue(new InstantCommand(() -> {arm.setArmGoalPosition(ArmPosition.AMP);}));

    // // Operator DPad Right: Set Arm Speaker Position 
    // new Trigger(() -> operatorController.getPOV() == 0)
    //   .onTrue(new ParallelCommandGroup ( new InstantCommand(() -> {
    //     arm.setArmGoalPosition(ArmPosition.SPEAKER);}),
    //     new SetDuring(4, LEDColor.PARTYMODE)
    //   ))
    //   .onFalse(new BlinkAlliance());
      

    // Operator DPad Right: Set Arm Speaker Position 
    // new Trigger(() -> operatorController.getPOV() == 45)
    //   .onTrue(new InstantCommand(() -> {arm.setArmGoalPosition(ArmPosition.TRAP);}));

    // // Operator DPad Left: Set Arm Rest Position
    // new Trigger(() -> operatorController.getPOV() == 270)
    //   // .onTrue(new SetArmPosition(arm, 0.2, false, ArmConstants.REST));
    //   .onTrue(new InstantCommand(() -> {arm.setArmGoalPosition(ArmPosition.PASSING);}));

    // new Trigger(() -> operatorController.getYButton())
    //     .onTrue(new InstantCommand(() -> {arm.setArmGoalPosition(ArmPosition.WHATEVER);}));

    // new Trigger(() -> operatorController.getLeftBumper())
    //   .onTrue(new RevShooter(shooter,4500, 4500)) // 51 42
    //   .onFalse(new InstantCommand(() -> {
    //   shooter.shooterSetOpenLoop(0, 0);
    // }, shooter));

    // new Trigger(() -> operatorController.getRightBumper())
    //   .onTrue(new ArmViaLimelight(limelight, arm, 1));

    // new Trigger(() -> operatorController.getRightStickButton())
    //   .onTrue(new InstantCommand(() -> {lift.setLeftGoalPosition(lift.getFINAL_POSITION_LEFT()); lift.setRightGoalPosition(lift.getFINAL_POSITION_RIGHT()); lift.setBreak(false);}));

    // new Trigger(() -> operatorController.getLeftStickButton())
    //   .onTrue(new InstantCommand(() -> {lift.setLeftGoalPosition(lift.getSTARTING_POSITION_LEFT()); lift.setRightGoalPosition(lift.getSTARTING_POSITION_RIGHT()); lift.setBreak(false);}));

    // new Trigger(() -> operatorController.getBackButton())
    //   .onTrue(new InstantCommand(() -> {lift.setLeftGoalPosition(LiftConstants.FINAL_POSITION_LEFT); lift.setRightGoalPosition(LiftConstants.FINAL_POSITION_RIGHT);}));
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

