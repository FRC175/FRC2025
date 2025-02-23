package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.servohub.ServoHub;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.ConfigurationFailedException;
import frc.robot.Constants.elevatorSetpoint;


public class Elevator extends SubsystemBase {
    private static Elevator instance;
    private final SparkMax master, slave;
    private final SparkMaxConfig defaultConfig;
    private final ResetMode resetMode;
    private final PersistMode persistMode;
    private final LaserCan distSensor;
    private elevatorSetpoint goalPoint;
    

    public Elevator() {
        this.master = new SparkMax(15, MotorType.kBrushless);
        this.slave = new SparkMax(16, MotorType.kBrushless);
        this.defaultConfig = new SparkMaxConfig();
        this.resetMode = SparkBase.ResetMode.kResetSafeParameters;
        this.persistMode = PersistMode.kPersistParameters;
        this.distSensor = new LaserCan(21);



        defaultConfig
        .inverted(false);
        configureSparks();
        configureDistSensor();

       goalPoint =  elevatorSetpoint.GROUND;
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
        master.configure(defaultConfig, resetMode, persistMode);
        // configure sparkMAX motor controllers
    }
    
    public void configureDistSensor () {
        try {
            distSensor.setRangingMode(LaserCan.RangingMode.LONG);
            distSensor.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
            distSensor.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
          } catch (ConfigurationFailedException e) {
            System.out.println("!LaserCAN config failed! " + e);
          }

          // attempts to configure LaserCAN, if the configuration fails, it prints an error message
    }
    
    public double getDistance() {
        LaserCan.Measurement measurement = distSensor.getMeasurement();
        return (measurement.distance_mm + -240) ;
    }

    public void setOpenLoop (double demand) {
        master.set(demand);
        slave.set(demand);
    }

    public elevatorSetpoint getGoalSetpoint () {
        return goalPoint;
    }

    public void setGoalPoint (elevatorSetpoint setpoint) {
        goalPoint = setpoint;
    }



   


}
