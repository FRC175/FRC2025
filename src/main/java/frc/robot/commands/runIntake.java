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
    private boolean upStream, downStream, firstIn ;
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
      firstIn = true;
      upStream = false;
      downStream = false;
      //intake.setState(intakePoints.OFF);
    }

    @Override
    public void execute() {
      boolean upstream = intake.isUpstream();
      boolean downStream = intake.isDownstream();
      intakePoints state = intake.getState();
      if (state == intakePoints.INTAKE_CORAL) {
        
        intake.setIntakeOpenLoop(-.1);  // was -.2
        if (downStream) {
          intake.setState(intakePoints.CAPTURED);
        }
      }
      if (state == intakePoints.CAPTURED) {
        if (downStream) {
          intake.setIntakeOpenLoop(.1);
        } else {
          intake.setIntakeOpenLoop (0);
        }
        
      }
      if (state == intakePoints.DISCHARGE_CORAL) {
        
          intake.setIntakeOpenLoop(-.2); 
        }
        
      
      if (state == intakePoints.OFF) {
        if (state == intakePoints.INTAKE_ALGAE) {
          firstIn = !firstIn;
        }
        intake.setIntakeOpenLoop(0);
      }

      if (state == intakePoints.INTAKE_ALGAE) {
          intake.setIntakeOpenLoop(.2);

      }

      if (state == intakePoints.HOLD_ALGAE) {
        intake.setIntakeOpenLoop(.085);

    }

      if (state == intakePoints.DISCHARGE_ALGAE) {
        intake.setIntakeOpenLoop(-.7);
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
