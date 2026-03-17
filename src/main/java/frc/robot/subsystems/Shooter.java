package frc.robot.subsystems;


import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import frc.robot.generated.constants;


public class Shooter extends SubsystemBase {


    // --- SHOOTER HARDWARE ---
    private final TalonFX m_leftMotor = new TalonFX(constants.kShooter_LeftCanId);
    private final TalonFX m_rightMotor = new TalonFX(constants.kShooter_RightCanId);
    private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0);


    // --- STATE VARIABLES ---
    private double m_targetRPM = 0;
   
    // --- INTERPOLATION MAP ---
    private final InterpolatingDoubleTreeMap m_rpmMap = new InterpolatingDoubleTreeMap();


    public Shooter() {
        // --- MOTOR CONFIGURATION ---
        TalonFXConfiguration config = new TalonFXConfiguration();


        // 1. Protect the motor (Internal torque limit)
        config.CurrentLimits.StatorCurrentLimit = 80.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;


        // 1. Protect the motor (Internal torque limit)
        config.CurrentLimits.StatorCurrentLimit = 80.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;


        // 2. Protect the battery (Breaker draw limit) - PREVENTS BROWNOUTS
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 60.0; // Allow a brief 60A spike...
        config.CurrentLimits.SupplyCurrentLowerTime = 0.1; // ...but only for 0.1 seconds...
        config.CurrentLimits.SupplyCurrentLowerLimit = 40.0; // ...before clamping down to a 40A continuous draw
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;


        config.Slot0.kP = constants.kShooter_kP;
        config.Slot0.kV = constants.kShooter_kV;


        m_leftMotor.getConfigurator().apply(config);


        m_rightMotor.setControl(
                new Follower(
                        m_leftMotor.getDeviceID(),
                        MotorAlignmentValue.Opposed));


        // --- DISTANCE (Meters) -> TARGET RPM ---
        // Note: You will tune these actual numbers on the field!
        m_rpmMap.put(1.5, 2800.0);
        m_rpmMap.put(2.0, 3200.0);
        m_rpmMap.put(2.5, 3600.0);
        m_rpmMap.put(3.0, 4100.0);
        m_rpmMap.put(3.5, 4600.0);
        m_rpmMap.put(4.0, 5200.0);
    }


    // ==========================================================
    // SHOOTER CONTROL METHODS
    // ==========================================================


    public void setRPM(double targetRPM) {
        m_targetRPM = targetRPM;
        double targetRPS = targetRPM / 60.0;
        m_leftMotor.setControl(m_velocityRequest.withVelocity(targetRPS));
    }


    /** Sets RPM using the manually tuned interpolation map */
    public void setRPMFromDistance(double distanceMeters) {
        // Clamping ensures we don't ask the map for a distance way outside our data
        distanceMeters = MathUtil.clamp(distanceMeters, 1.5, 4.0);
        double targetRPM = m_rpmMap.get(distanceMeters);
        setRPM(targetRPM);
    }


    public double getCurrentRPM() {
        return m_leftMotor.getVelocity()
                .refresh()
                .getValueAsDouble() * 60.0;
    }


    public boolean isAtSpeed() {
        return Math.abs(getCurrentRPM() - m_targetRPM)
                < constants.kShooterToleranceRPM;
    }


    public void stop() {
        m_targetRPM = 0;
        m_leftMotor.stopMotor();
    }
}
