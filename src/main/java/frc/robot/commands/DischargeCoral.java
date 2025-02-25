package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Elevator;
import frc.robot.Constants.ManipConstants;

public class DischargeCoral extends Command{
    private final Manipulator manipulator;
    private final Elevator elevator;
    private boolean discharging, downStream;
    private double demand;
   
    
   
    

    
    public DischargeCoral(double demand) {
      this.manipulator = Manipulator.getInstance();
      this.elevator = Elevator.getInstance();
      discharging = false;
      downStream = false;
      this.demand = -demand;
      

      
     
    }

    @Override
    public void execute() {
        elevator.coralInPeril = true;
        manipulator.setIntakeOpenLoop(demand);
        if (downStream) {
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
     return (!downStream && discharging);
    }

}
