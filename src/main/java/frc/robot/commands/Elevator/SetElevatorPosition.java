package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Manipulator;
import frc.robot.Constants.manipulatorSetpoint;
import frc.robot.Constants.ElevatorSetpoint;
import frc.robot.subsystems.Elevator;

public class SetElevatorPosition extends SequentialCommandGroup {
    private ElevatorSetpoint setp, prevSetp;
    public SetElevatorPosition(Manipulator manipulator, Elevator elevator, ElevatorSetpoint setpoint) {
        this.setp = setpoint;
        
        addCommands(
            new InstantCommand(() -> {
                if (manipulator.getEncoder() < manipulatorSetpoint.CORALTRAVEL.getSetpoint()) {
                    manipulator.setGoalPoint(manipulatorSetpoint.CORALTRAVEL.getSetpoint());
                }
            }),
            new InstantCommand(() -> { if (manipulator.getEncoder() > .5) {
                if (setpoint == ElevatorSetpoint.L1) {
                    setp = ElevatorSetpoint.PROCESSOR;
                }
                if (setpoint == ElevatorSetpoint.L2) {
                    setp = ElevatorSetpoint.BTM_ALGAE;
                }
                if (setpoint == ElevatorSetpoint.L3) {
                    setp = ElevatorSetpoint.TOP_ALGAE;
                }
            } else {
                if (setpoint == ElevatorSetpoint.L1) {
                    setp = ElevatorSetpoint.L1;
                }
                if (setpoint == ElevatorSetpoint.L2) {
                    setp = ElevatorSetpoint.L2;
                }
                if (setpoint == ElevatorSetpoint.L3) {
                    setp = ElevatorSetpoint.L3;
                }
            }}),
            new WaitUntilCommand(manipulator.isAtGoal(.011)),
            new InstantCommand(() -> { elevator.setGoalPoint(setp);}),
            new WaitUntilCommand(elevator.isAtGoal(100)),
            new InstantCommand(() -> {
                if (setp == ElevatorSetpoint.L1) {
                    manipulator.setGoalPoint(manipulatorSetpoint.CORALIN.getSetpoint());
                }
                if (setp == ElevatorSetpoint.PROCESSOR) {
                    manipulator.setGoalPoint(manipulatorSetpoint.PROCESSOR.getSetpoint());
                }
                if (setp == ElevatorSetpoint.BTM_ALGAE || setp == ElevatorSetpoint.TOP_ALGAE) {
                    manipulator.setGoalPoint(manipulatorSetpoint.ALGAEIN.getSetpoint());
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
    