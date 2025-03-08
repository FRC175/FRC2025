package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkMaxAlternateEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
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




public class Intake extends SubsystemBase {
    
    private static Intake instance;
    private final SparkMax intake;
    private final SparkMaxConfig defaultConfig;
    private final ResetMode resetMode;
    private final PersistMode persistMode;
    private boolean isFlipped;
    public boolean manual, cc;
    private AbsoluteEncoderConfig flipConfig;
    private SparkAbsoluteEncoder flipEncoder;
    private DigitalInput upstream, downstream;
    private manipulatorSetpoint currentSetpoint, goalPosition;
    private double goalPoint;
    private intakePoints state;
    // default position (false) is coral

    public Intake() {
        
        this.intake = new SparkMax(18, MotorType.kBrushless);
        this.defaultConfig = new SparkMaxConfig();
        this.downstream = new DigitalInput(1);
        this.upstream = new DigitalInput(2);
        resetMode = SparkBase.ResetMode.kResetSafeParameters;
        persistMode = PersistMode.kPersistParameters;
        manual = false;
        cc = false;
        state = intakePoints.OFF;
        
        isFlipped = false;

        defaultConfig
        .inverted(false);
        configureSparks(defaultConfig, resetMode, persistMode);
        goalPoint = manipulatorSetpoint.CORALTRAVEL.getSetpoint();
        // goalPosition = manipulatorSetpoint.CORALIN;
        // currentSetpoint = manipulatorSetpoint.CORALIN;
    }
    
    @Override
    public void periodic() {
       //SmartDashboard.putNumber("flipAngle", getEncoder());
        //SmartDashboard.putNumber("motor demand",)
        SmartDashboard.putBoolean("upstream", upstream.get());
        SmartDashboard.putBoolean("downstream", downstream.get());
        //System.out.println("upstream: " + upstream.get());
        // System.out.println("downstream: " + upstream.get());
    }

    public static Intake getInstance() {
        if ( instance == null) {
            instance = new Intake();
        }
        return instance;
    }

    public intakePoints getState () {
        return state;
    }

    public void setState(intakePoints state) {
        this.state = state;
    }

   
   
    public void setIntakeOpenLoop (double demand) {
        intake.set(demand);
    }

    public boolean isUpstream() {
        return !upstream.get();
    }

    public double getIntakeSpeed() {
        RelativeEncoder encoder = intake.getEncoder();
        return encoder.getVelocity();

    }

    public boolean isDownstream() {
        return !downstream.get();
    }

   



    private void configureSparks (SparkMaxConfig config, SparkBase.ResetMode resetMode, PersistMode persistmode) {
        intake.configure(config, resetMode, persistMode);
    }

    

    

}
