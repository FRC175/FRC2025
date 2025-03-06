package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.elevatorSetpoint;

public class SetElevatorPosition extends Command{
    private final Elevator elevator;
    private double goalPoint;
    private double upSpeed, downSpeed, deadband, dist;
    private elevatorSetpoint goalSet;

    
    public SetElevatorPosition (double upSpeed, double downSpeed, double deadband, elevatorSetpoint setpoint) {
        this.elevator = Elevator.getInstance();
        goalPoint = setpoint.getSetpoint();
        this.deadband = deadband;
        this.downSpeed = downSpeed;
        this.upSpeed = upSpeed;

        

    }


    @Override
    public void execute() {
      if (!elevator.coralInPeril || elevator.coralOverride) {

      
          dist = elevator.getDistance();

        if (dist - deadband <= goalPoint){
            elevator.setOpenLoop(upSpeed);
        }
        if (dist + deadband <= goalPoint) {
            elevator.setOpenLoop(-downSpeed);

        }

        


      }
    }
    
    @Override
    public void end(boolean interrupted) {
      System.out.println(goalPoint + " stopped");
        elevator.setOpenLoop(0);
        
    }
    @Override
    public boolean isFinished() {
      double max = ElevatorConstants.MAX_HEIGHT;
      double min = ElevatorConstants.MIN_HEIGHT;
      if (dist - 10 <= goalPoint || dist + 10 >= goalPoint || dist >= max || dist <= min) {
        return true;
      } else {
        return false;
      }
    }

}
