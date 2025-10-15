package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Manipulator;

public class ControlElevator extends Command {
    private final Elevator elevator;
    private double goalPoint;
    private double upSpeed, downSpeed, deadband;

    public ControlElevator (double upSpeed, double downSpeed, double deadband) {
        this.elevator = Elevator.getInstance();
        goalPoint = elevator.getGoalSetpoint();
        this.deadband = deadband;
        this.downSpeed = downSpeed;
        this.upSpeed = upSpeed;
        addRequirements(Elevator.getInstance());

    }

    @Override
    public void execute() {
      Manipulator manipulator = Manipulator.getInstance();
      
      if (manipulator.isInDangerZone()) {
        elevator.setOpenLoop(0);
      }
      

      goalPoint = elevator.getGoalSetpoint();
      double dist = elevator.getDistance();
      if (dist < goalPoint - deadband) {
        elevator.setOpenLoop(-upSpeed);
      } else if (dist > goalPoint + deadband) {
        elevator.setOpenLoop(downSpeed);
      } else {
        elevator.setOpenLoop(calculateProportionalOutput(dist, goalPoint));
      }
    
    }

    private double calculateProportionalOutput(double dist, double goalPoint) {
      double slope = (downSpeed + upSpeed) / (8 * deadband);
      double output = (dist - goalPoint) * slope;
      return output;
    }
    
    @Override
    public void end(boolean interrupted) {
      elevator.setOpenLoop(0);
    }

    @Override
    public boolean isFinished() {
      return false;   // this runs *forever*
    }

    
}
