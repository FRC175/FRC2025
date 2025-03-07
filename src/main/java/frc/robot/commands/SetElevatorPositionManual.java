package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.Constants.ElevatorConstants;

public class SetElevatorPositionManual extends Command{
    private final Elevator elevator;
    private double speed, increment, goalPoint;

    public SetElevatorPositionManual (double speed, double increment) {
        this.elevator = Elevator.getInstance();
        this.increment = increment;
        this.speed = speed;
        this.goalPoint = ElevatorConstants.MIN_HEIGHT;  // just to give this a value, will be set correctly on init
    }

    @Override
    public void initialize() {
      this.goalPoint = elevator.getDistance() + increment;
    }

    @Override
    public void execute() {
      double dist = elevator.getDistance();
      if (dist <= goalPoint){
        elevator.setOpenLoop(speed);
      } else if (dist >= goalPoint) {
          elevator.setOpenLoop(speed);
      }
    }
    
    @Override
    public void end(boolean interrupted) {
        elevator.setOpenLoop(0);
    }

    @Override
    public boolean isFinished() {
      double dist = elevator.getDistance();
      boolean goingUp = increment > 0.0;
      return (goingUp && dist >= goalPoint) || (!goingUp && dist <= goalPoint);
    }
}
