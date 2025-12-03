package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.servohub.ServoHub;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.ConfigurationFailedException;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Elevator extends SubsystemBase {
    private static Elevator instance;
    private final SparkFlex right, left;
    private final SparkFlexConfig defaultConfig;
    private final ResetMode resetMode;
    private final PersistMode persistMode;
    private final LaserCan distSensor;
    public boolean coralInPeril;
    public boolean coralOverride;
    public boolean manual;
  
    private final DigitalInput topProxSwitch, botProxSwitch;
    

    public Elevator() {
        this.right = new SparkFlex(3, MotorType.kBrushless);
        this.left = new SparkFlex(2, MotorType.kBrushless);
        this.defaultConfig = new SparkFlexConfig();
        this.resetMode = SparkBase.ResetMode.kResetSafeParameters;
        this.persistMode = PersistMode.kPersistParameters;
        this.distSensor = new LaserCan(21);
        coralOverride = false;
        this.botProxSwitch = new DigitalInput(3);
        this.topProxSwitch = new DigitalInput(4);



        defaultConfig
        .inverted(false);
        configureSparks();
       
    }
    
    @Override
    public void periodic() {
       
    }

    

    public static Elevator getInstance() {
        if ( instance == null) {
            instance = new Elevator();
        }
        return instance;
        // if an instance of an Elavator already exists, it is returned. if not, a new one is created.
    }

    public void configureSparks () {
        right.configure(defaultConfig, resetMode, persistMode);
        left.configure(defaultConfig, resetMode, persistMode);
        // configure sparkMAX motor controllers
    }
   
    public void setOpenLoopR (double demand) {
        right.set(demand);
    }

    public void setOpenLoopL (double demand) {
        left.set(demand);
    }
   



   


}
