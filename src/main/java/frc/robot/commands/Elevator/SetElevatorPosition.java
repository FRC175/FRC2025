package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Manipulator;
import frc.robot.Constants.manipulatorSetpoint;
import frc.robot.Constants.ElevatorSetpoint;
import frc.robot.subsystems.Elevator;

public class SetElevatorPosition extends SequentialCommandGroup {

    public SetElevatorPosition(Manipulator manipulator, Elevator elevator, ElevatorSetpoint setpoint) {
        addCommands(
            new InstantCommand(() -> {
                if (manipulator.getEncoder() < manipulatorSetpoint.CORALTRAVEL.getSetpoint()) {
                    manipulator.setGoalPoint(manipulatorSetpoint.CORALTRAVEL.getSetpoint());
                }
            }),
            new WaitUntilCommand(manipulator.isAtGoal(.28)),
            new InstantCommand(() -> { elevator.setGoalPoint(setpoint); }),
            new InstantCommand(() -> {
                if (setpoint == ElevatorSetpoint.L1) {
                    manipulator.setGoalPoint(manipulatorSetpoint.CORALIN.getSetpoint());
                }
            })
            // new InstantCommand(() -> {
            //     if (setpoint == ElevatorSetpoint.L4) {
            //         manipulator.setGoalPoint(manipulatorSetpoint.L4CORAL.getSetpoint());
            //     }
            // }) 
        );

        
    }
}
    