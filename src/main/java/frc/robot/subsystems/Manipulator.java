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



public class Manipulator extends SubsystemBase {
    private static Manipulator instance;
    private final SparkMax flip, intake;
    private final SparkMaxConfig defaultConfig;
    private final ResetMode resetMode;
    private final PersistMode persistMode;
    private final LaserCan distSensor;
    private elevatorSetpoint goalPoint;
    private bool isFlipped;
    // default position (0) is coral

    public Manipulator() {
        this.flip = new SparkMax(17, Motortype.kBrushless);
        this.intake = new SparkMax(18, Motortype.kBrushless);
        this.defaultConfig = new SparkMaxConfig();
        this.resetMode = SparkBase.ResetMode.kResetSafeParameters;
        this.persistMode = PersistMode.kPersistParameters;
        isFlipped = 0;

        defaultConfig
        .inverted(false);
        configureSparks();
        configureDistSensor();

    }
    
    @Override
    public void periodic() {

    }

    public static Manipulator getInstance() {
        if ( instance == null) {
            instance = new Manipulatorulator();
        }
        return instance;
    }
    
    public boolean isFlipped() {
        return isFlipped;
    }

    private void flipIntake () {
        SparkMaxConfig flipConfig = new SparkMaxConfig
        config
        .inverted(true)
        flip.configure(flipConfig, resetMode, persistMode);
    }

    


    private void configureSparks(SparkMaxConfig config, SparkBase.resetMode resetMode, PersistMode persistmode) {
        flip.configure(config, resetMode, persistMode);
        intake.configure(config, resetMode, persistMode);
    }

    

}
