package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ElevatorSetpoint;
import frc.robot.Constants.intakePoints;
import frc.robot.commands.Drive.SwerveToDist;
import frc.robot.commands.Elevator.SetElevatorPosition;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Drive.Drive;
import frc.robot.utils.Utils;

public class B2L4 extends SequentialCommandGroup {

    public B2L4(Drive drive, Intake intake, Manipulator manipulator, Elevator elevator) {
        addCommands(
        new InstantCommand(() -> drive.resetGyro(180)),
        new InstantCommand(() -> intake.setState(intakePoints.INTAKE_CORAL)),
        new WaitUntilCommand(intake.isCaptured()),
        new SwerveToDist(drive, .3, 180, 180, 3 ),
        new SetElevatorPosition(manipulator, elevator, ElevatorSetpoint.L4),
        new SwerveToDist(drive, .3, 180, 180, .1),
        new InstantCommand(() -> intake.setState(intakePoints.DISCHARGE_CORAL)),
        new WaitCommand(4),
        new InstantCommand(() -> intake.setState(intakePoints.OFF))
        );
        

        
    }
    
}