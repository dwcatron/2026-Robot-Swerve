package frc.robot.subsystems;


import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.constants;


public class Intake extends SubsystemBase {


    // --- ORIGINAL FRONT INTAKE MOTORS ---
    private final SparkMax m_FrontMotor = new SparkMax(constants.kFrontIntakeCanId, MotorType.kBrushless);
    private final SparkMax m_RearMotor = new SparkMax(constants.kRearIntakeCanId, MotorType.kBrushless);
   
    // --- NEW DEPLOYABLE BACK INTAKE MOTOR ---
    private final SparkMax m_hopperintake = new SparkMax(constants.kHopperIntakeCanId, MotorType.kBrushless);


    public Intake() {
        SparkMaxConfig config = new SparkMaxConfig();
       
        // I uncommented this because Coast mode and current limits are highly recommended for intakes!
        config.smartCurrentLimit(40).idleMode(IdleMode.kCoast);


        // Apply config to all 3 motors
        m_FrontMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_RearMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_hopperintake.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }


    /** Runs the main front intake (which uses the Front and Rear motors together) */
    public void setSpeed(double front, double rear) {
        m_FrontMotor.set(front);
        m_RearMotor.set(rear);
    }


    /** Runs the new 3rd motor on the deployable back intake */
    public void setHopperIntakeSpeed(double speed) {
        m_hopperintake.set(speed);
    }


    /** Stops all three intake motors */
    public void stop() {
        m_FrontMotor.set(0);
        m_RearMotor.set(0);
        m_hopperintake.set(0);
    }


    // --- CURRENT MONITORING METHODS ---
    public double getFrontCurrent() {
        return m_FrontMotor.getOutputCurrent();
    }


    public double getRearCurrent() {
        return m_RearMotor.getOutputCurrent();
    }


    public double getHopperIntakeCurrent() {
        return m_hopperintake.getOutputCurrent();
    }


    /** Feeds the game piece into the shooter */
    public void feedShooter() {
        // Run the intake motors to push the game piece into the shooter
        // (Adjust the 1.0 down if it jams!)
        setSpeed(1.0, 1.0);
    }
}
