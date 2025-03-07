package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.Constants.ElevatorSetpoint;

public class SetElevatorPosition extends Command {
    private final Elevator elevator;
    private double goalPoint;
    private double upSpeed, downSpeed, deadband;

    public SetElevatorPosition (double upSpeed, double downSpeed, double deadband, ElevatorSetpoint setpoint) {
        this.elevator = Elevator.getInstance();
        goalPoint = setpoint.getSetpoint();
        this.deadband = deadband;
        this.downSpeed = downSpeed;
        this.upSpeed = upSpeed;
    }

    @Override
    public void execute() {
      // if (!elevator.coralInPeril || elevator.coralOverride) {
      //   double dist = elevator.getDistance();

      //   if (dist - deadband <= goalPoint){
      //       elevator.setOpenLoop(upSpeed);
      //   }
      //   if (dist + deadband <= goalPoint) {
      //       elevator.setOpenLoop(-downSpeed);
      //   }
      // }

      double dist = elevator.getDistance();
      if (dist <= goalPoint) {
        System.out.println("Driving up");
        elevator.setOpenLoop(-upSpeed);
      } else if (dist >= goalPoint) {
        System.out.println("Driving down");
        elevator.setOpenLoop(downSpeed);
      }
    }
    
    @Override
    public void end(boolean interrupted) {
      System.out.println(goalPoint + " stopped");
      elevator.setOpenLoop(0);
    }

    @Override
    public boolean isFinished() {
      // double max = ElevatorConstants.MAX_HEIGHT;
      // double min = ElevatorConstants.MIN_HEIGHT;
      // if (dist - 10 <= goalPoint || dist + 10 >= goalPoint || dist >= max || dist <= min) {
      //   return true;
      // } else {
      //   return false;
      // }

      double dist = elevator.getDistance();
      return Math.abs(dist - goalPoint) <= deadband;
    }
}
