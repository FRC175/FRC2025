package frc.robot.commands.manipulator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Elevator;
import frc.robot.Constants.ManipConstants;

public class Discharge extends Command{
    private final Manipulator manipulator;
    private final Elevator elevator;
    private boolean discharging, downStream;
    private double demand;
   
    
   
    

    
    public Discharge(double demand) {
      this.manipulator = Manipulator.getInstance();
      this.elevator = Elevator.getInstance();
      discharging = false;
      downStream = false;
      this.demand = -demand;
      

      
     
    }

    @Override
    public void execute() {
      if (!manipulator.isFlipped()) {
        elevator.coralInPeril = true;
        manipulator.setIntakeOpenLoop(demand);
        if (downStream) {
          discharging = true;
        }
      } else {
        manipulator.setIntakeOpenLoop(demand);
          discharging = true;
      }
      
    }
    
    @Override
    public void end(boolean interrupted) {
       elevator.coralInPeril = false;
        manipulator.setIntakeOpenLoop(0);
        
    }
    @Override
    public boolean isFinished() {
      if( !manipulator.isFlipped()) {
     return (!downStream && discharging);
      } else {
        return false;
      }
    }

}
