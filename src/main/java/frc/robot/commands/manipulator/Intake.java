package frc.robot.commands.manipulator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Elevator;
import frc.robot.Constants.ManipConstants;

public class Intake extends Command{
    private final Manipulator manipulator;
    private final Elevator elevator;
    private boolean adjusting, upStream, downStream, algae;
    private double demand;
    
   
    
   
    

    
    public Intake(double demand) {
      this.manipulator = Manipulator.getInstance();
      this.elevator = Elevator.getInstance();
      this.demand = demand;
      
      addRequirements(manipulator);
    }

    @Override
    public void initialize() {
      adjusting = false;
      upStream = false;
      downStream = false;
    }

    @Override
    public void execute() {
      if (manipulator.getEncoder() > .5) algae = true; else algae = false;
      if (!algae) {
        boolean upstream = manipulator.isUpstream();
        boolean downStream = manipulator.isDownstream();
        elevator.coralInPeril = true;
        if (!adjusting) {
          manipulator.setIntakeOpenLoop(demand);
        }
        if (upstream) {
          manipulator.setIntakeOpenLoop(demand);
        }
        if (downStream) {
          manipulator.setIntakeOpenLoop(-demand); 
          adjusting = true;
        } 
      } else {
        manipulator.setIntakeOpenLoop(demand);
        adjusting = true;
      }
    }
    
    @Override
    public void end(boolean interrupted) {
      //System.out.println("code sucks");
      elevator.coralInPeril = false;
      manipulator.setIntakeOpenLoop(0);
    }

    @Override
    public boolean isFinished() {
      if (!algae) {
        return (!upStream && !downStream && adjusting);
      } else {
        return (adjusting && manipulator.getIntakeSpeed() <= .1);
      }
      
    }

}
