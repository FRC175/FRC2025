package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.Constants.ElevatorConstants;

public class SetElevatorPositionManual extends Command{
    private final Elevator elevator;
    private boolean goingUp;
    private double speed, dist;

    
    public SetElevatorPositionManual (boolean goingUp, double speed) {
        this.elevator = Elevator.getInstance();
        this.goingUp = goingUp;
        this.speed = speed;

        
        

        

    }

    @Override
    public void execute() {
      dist = elevator.getDistance();
      elevator.setOpenLoop(speed);
    }
    
    @Override
    public void end(boolean interrupted) {
        elevator.setOpenLoop(-.1);
        System.out.println("over");
        
    }
    @Override
    public boolean isFinished() {

      double maxheight = ElevatorConstants.MAX_HEIGHT;
      double minHeight = ElevatorConstants.MIN_HEIGHT;
      return (dist >= maxheight|| dist <= minHeight );
    }

}
