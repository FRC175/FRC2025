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
import frc.robot.Constants.elevatorSetpoint;
import frc.robot.Constants.manipulatorSetpoint;




public class Manipulator extends SubsystemBase {
    
    
    private static Manipulator instance;
    private final SparkMax flip, intake;
    private final SparkMaxConfig defaultConfig;
    private final ResetMode resetMode;
    private final PersistMode persistMode;
    private boolean isFlipped;
    public boolean manual, cc;
    private AbsoluteEncoderConfig flipConfig;
    private SparkAbsoluteEncoder flipEncoder;
    private DigitalInput upstream, downstream;
    private manipulatorSetpoint currentSetpoint, goalPosition;
    // default position (false) is coral

    public Manipulator() {
        this.flip = new SparkMax(17, MotorType.kBrushless);
        this.intake = new SparkMax(18, MotorType.kBrushless);
        this.defaultConfig = new SparkMaxConfig();
        this.flipEncoder = flip.getAbsoluteEncoder();
        this.flipConfig = new AbsoluteEncoderConfig();
        this.downstream = new DigitalInput(1);
        this.upstream = new DigitalInput(2);
        resetMode = SparkBase.ResetMode.kResetSafeParameters;
        persistMode = PersistMode.kPersistParameters;
        manual = false;
        cc = false;
        
        isFlipped = false;

        defaultConfig
        .inverted(false);
        configureSparks(defaultConfig, resetMode, persistMode);
        flipConfig
        .inverted(false)
        .setSparkMaxDataPortConfig();
        
        goalPosition = manipulatorSetpoint.CORAL;
        currentSetpoint = manipulatorSetpoint.CORAL;
    }
    
    @Override
    public void periodic() {
        SmartDashboard.putNumber("manip Encoder", getEncoder());
        //SmartDashboard.putNumber("motor demand",)
    //     System.out.println("upstream: " + upstream.get());
    //     System.out.println("downstream: " + upstream.get());
    }

    public static Manipulator getInstance() {
        if ( instance == null) {
            instance = new Manipulator();
        }
        return instance;
    }
    
    public boolean isFlipped() {
        return isFlipped;
    }

    public void setFlipped(boolean bool) {
        isFlipped = bool;
    }

    public void invertFlip () {
       SparkMaxConfig invertConfig = new SparkMaxConfig();
        invertConfig
        .inverted(isFlipped);
        flip.configure(invertConfig, resetMode, persistMode);
     }

    public void setIntakeOpenLoop (double demand) {
        intake.set(demand);
    }

    public void setFlipOpenLoop (double demand) {
        flip.set(demand);
    }

    public double getEncoder() {
        return flipEncoder.getPosition();
    }

    public boolean isUpstream() {
        return upstream.get();
    }

    public double getIntakeSpeed() {
        RelativeEncoder encoder = intake.getEncoder();
        return encoder.getVelocity();

    }

    public boolean isDownstream() {
        return downstream.get();
    }

    public manipulatorSetpoint getGoalSetpoint() {
        return goalPosition;
    }

    public void setGoalSetpoint(manipulatorSetpoint setpoint) {
        goalPosition = setpoint;
    }

    public manipulatorSetpoint getCurrentSetpoint() {
        return currentSetpoint;
    }

    public void setCurrentSetpoint(manipulatorSetpoint  setpoint) {
        currentSetpoint = setpoint;
    }




    private void configureSparks (SparkMaxConfig config, SparkBase.ResetMode resetMode, PersistMode persistmode) {
        flip.configure(config, resetMode, persistMode);
        intake.configure(config, resetMode, persistMode);
    }

    

    

}
