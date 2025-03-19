package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.Constants.ManipConstants;
import frc.robot.Constants.intakePoints;

public class runIntake extends Command{
    private final Manipulator manipulator;
    private final Elevator elevator;
    private final Intake intake;
    private boolean adjusting, upStream, downStream, algae;
    private double demand;
    
   
    
   
    

    
    public runIntake(double demand) {
      this.manipulator = Manipulator.getInstance();
      this.elevator = Elevator.getInstance();
      this.demand = demand;
      this.intake = Intake.getInstance();
      
      addRequirements(intake);
    }

    @Override
    public void initialize() {
      adjusting = false;
      upStream = false;
      downStream = false;
      intake.setState(intakePoints.OFF);
    }

    @Override
    public void execute() {
      boolean upstream = intake.isUpstream();
      boolean downStream = intake.isDownstream();
      intakePoints state = intake.getState();
      if (state == intakePoints.INTAKE_CORAL) {
        
        intake.setIntakeOpenLoop(-demand);
        if (downStream) {
          intake.setState(intakePoints.CAPTURED);
        }
      }
      if (state == intakePoints.CAPTURED) {
        if (downStream) {
          intake.setIntakeOpenLoop(demand);
        } else {
          intake.setIntakeOpenLoop (0);
        }
        
      }
      if (state == intakePoints.DISCHARGE_CORAL) {
        if (manipulator.getEncoder() > .5) {
          intake.setIntakeOpenLoop(-1);
        } else {
          intake.setIntakeOpenLoop(-demand); 
          
        }
        }
        
      
      if (state == intakePoints.OFF) {
        intake.setIntakeOpenLoop(0);
      }
      if (state == intakePoints.INTAKE_ALGAE) {
        intake.setIntakeOpenLoop(-demand);
      }
      if (state == intakePoints.DISCHARGEALGAE) {
        intake.setIntakeOpenLoop(1);
      }
      
      // if (manipulator.getEncoder() > .5) algae = true; else algae = false;
      // if (!algae) {
      //   boolean upstream = manipulator.isUpstream();
      //   boolean downStream = manipulator.isDownstream();
      //   elevator.coralInPeril = true;
      //   if (!adjusting) {
      //     manipulator.setIntakeOpenLoop(demand);
      //   }
      //   if (upstream) {
      //     manipulator.setIntakeOpenLoop(demand);
      //   }
      //   if (downStream) {
      //     manipulator.setIntakeOpenLoop(-demand); 
      //     adjusting = true;
      //   } 
      // } else {
      //   manipulator.setIntakeOpenLoop(demand);
      //   adjusting = true;
      // }
    }
    
    @Override
    public void end(boolean interrupted) {
    
      intake.setIntakeOpenLoop(0);
    }

    @Override
    public boolean isFinished() {
      return false;
      
    }


}
