package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.Constants.elevatorSetpoint;

public class SetElevatorPosition extends Command{
    private final Elevator elevator;
    private double goalPoint;
    private double upSpeed, downSpeed, deadband, dist;

    
    public SetElevatorPosition (double upSpeed, double downSpeed, double deadband) {
        this.elevator = Elevator.getInstance();
        goalPoint = elevator.getGoalSetpoint().getSetpoint();
        this.deadband = deadband;
        this.downSpeed = downSpeed;
        this.upSpeed = upSpeed;

        

    }

    @Override
    public void execute() {
        double dist = elevator.getDistance();

        if (dist - deadband <= goalPoint){
            elevator.setOpenLoop(upSpeed);
        }
        if (dist + deadband <= goalPoint) {
            elevator.setOpenLoop(-downSpeed);
        }

        


        super.execute();
    }
    
    @Override
    public void end(boolean interrupted) {
        elevator.setOpenLoop(0);
        
    }
    @Override
    public boolean isFinished() {
      if (dist - 10 <= goalPoint || dist + 10 >= goalPoint ) {
        return true;
      } else {
        return false;
      }
    }

}
