package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.units.PerUnit;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkMaxAlternateEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import java.util.function.BooleanSupplier;

import com.fasterxml.jackson.databind.node.DoubleNode;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.servohub.ServoHub;
import com.revrobotics.sim.SparkSimFaultManager;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.AlternateEncoderConfig;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ManipConstants;
import frc.robot.Constants.ElevatorSetpoint;
import frc.robot.Constants.manipulatorSetpoint;
import frc.robot.Constants.intakePoints;




public class Manipulator extends SubsystemBase {
    
    private static Manipulator instance;
    private final SparkMax flip;
    private final SparkMaxConfig defaultConfig;
    private final ResetMode resetMode;
    private final PersistMode persistMode;
    
    private boolean manual;
    private AbsoluteEncoderConfig flipConfig;
    private SparkAbsoluteEncoder flipEncoder;
    private DigitalInput upstream, downstream;
    private manipulatorSetpoint currentSetpoint, goalPosition;
    private double goalPoint;
    private intakePoints state;
    // default position (false) is coral

    public Manipulator() {
        this.flip = new SparkMax(17, MotorType.kBrushless);
        this.defaultConfig = new SparkMaxConfig();
        this.flipEncoder = flip.getAbsoluteEncoder();
        this.flipConfig = new AbsoluteEncoderConfig();
        resetMode = SparkBase.ResetMode.kResetSafeParameters;
        persistMode = PersistMode.kPersistParameters;
        manual = false;
        state = intakePoints.OFF;
        
        

        defaultConfig.inverted(false);
        defaultConfig.openLoopRampRate(.25);
        
        configureSparks(defaultConfig, resetMode, persistMode);
       
        goalPoint = manipulatorSetpoint.CORALTRAVEL.getSetpoint();
        // goalPosition = manipulatorSetpoint.CORALIN;
        // currentSetpoint = manipulatorSetpoint.CORALIN;
    }
    
    @Override
    public void periodic() {
       SmartDashboard.putNumber("flipAngle", getEncoder());
       SmartDashboard.putNumber("GoalAngle", getGoalSetpoint());
       //System.out.println(!((getEncoder() < getGoalSetpoint() - .05) || (getEncoder() > getGoalSetpoint() + .05)));
       //System.out.println(isAtGoal(.05).getAsBoolean());
       }

    public static Manipulator getInstance() {
        if ( instance == null) {
            instance = new Manipulator();
        }
        return instance;
    }

    public intakePoints getState () {
        return state;
    }

    public void setState(intakePoints state) {
        this.state = state;
    }

    public boolean isInDangerZone() {
        return (getEncoder() < manipulatorSetpoint.CORALTRAVEL.getSetpoint());
    }
  
    public void invertFlip () {
       SparkMaxConfig invertConfig = new SparkMaxConfig();
        invertConfig
        .inverted(false);
        flip.configure(invertConfig, resetMode, persistMode);
     }

    

    public void setFlipOpenLoop (double demand) {
        flip.set(demand);
    }

    public double getEncoder() {
        return flipEncoder.getPosition();
    }

   

    public double getGoalSetpoint() {
        return goalPoint;
    }

    public void setGoalPoint(double setpoint) {
        goalPoint = setpoint;
    }

    public manipulatorSetpoint getCurrentSetpoint() {
        return currentSetpoint;
    }

    public void setCurrentSetpoint(manipulatorSetpoint  setpoint) {
        currentSetpoint = setpoint;
    }




    private void configureSparks (SparkMaxConfig config, SparkBase.ResetMode resetMode, PersistMode persistmode) {
        flip.configure(config, resetMode, persistMode);
       
    }

    public BooleanSupplier isAtGoal(double deadband) {
        BooleanSupplier isAtGoal = () -> (!((getEncoder() < getGoalSetpoint() - deadband) || (getEncoder() > getGoalSetpoint() + deadband)));
       return isAtGoal;
    }

    

    

}
