package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.constants;

public class Hood extends SubsystemBase {
    private final SparkMax m_motor = new SparkMax(constants.kHoodCanId, MotorType.kBrushless);
    private final CANcoder m_encoder = new CANcoder(constants.kHoodEncoderCanId);

    private final PIDController m_pid = new PIDController(0.0001, 0, 0.001); 
    private double m_targetAngle = 0;
    private boolean m_isClosedLoop = false;

    // --- INTERPOLATION MAP ---
    private final InterpolatingDoubleTreeMap m_angleMap = new InterpolatingDoubleTreeMap();

    public Hood() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(20).idleMode(IdleMode.kBrake);
        
        m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_pid.setTolerance(0.5); 

        // --- DISTANCE (Meters) -> HOOD ANGLE (Degrees) ---
        // Match the distance keys from your Shooter map for easier tuning!
        m_angleMap.put(1.5, 45.0); // Close up: aim high
        m_angleMap.put(2.0, 38.0); 
        m_angleMap.put(2.5, 32.0); 
        m_angleMap.put(3.0, 25.0); 
        m_angleMap.put(3.5, 18.0); 
        m_angleMap.put(4.0, 12.0); // Far away: aim low/flat

        Shuffleboard.getTab("Tuning").add("Hood PID", m_pid);
    }
    
    @Override
    public void periodic() {
        double currentAngle = getAngle();

        if (m_isClosedLoop) {
            double output = m_pid.calculate(currentAngle, m_targetAngle);
            m_motor.set(output);
        }

        SmartDashboard.putNumber("Hood/Angle", currentAngle);
        SmartDashboard.putNumber("Hood/Raw Degrees", m_encoder.getAbsolutePosition().getValueAsDouble() * 360.0);
    }

    public boolean isAtPosition(double target, double tolerance) {
        return Math.abs(getAngle() - target) < tolerance;
    }

    /**
     * Default version: Checks if the hood is at the internal target angle 
     * with a default 1.0 degree tolerance.
     */
    public boolean isAtPosition() {
        return Math.abs(getAngle() - m_targetAngle) < 1.0; 
    }
    public void setAngleFromDistance(double distanceMeters) {
        // Clamp to match the map limits
        distanceMeters = MathUtil.clamp(distanceMeters, 1.5, 4.0);
        double targetAngle = m_angleMap.get(distanceMeters); 
        setAngle(targetAngle);
    }
    
    public void setAngle(double degrees) {
        m_targetAngle = degrees;
        m_isClosedLoop = true;
    }

    public void setMainMotorSpeed(double speed) {
        m_isClosedLoop = false; 
        m_motor.set(speed);
    }

    public double getAngle() {
        return (m_encoder.getAbsolutePosition().refresh().getValueAsDouble() * 360.0) - constants.kHoodOffset;
    }

    public void stop() {
        m_isClosedLoop = false;
        m_motor.stopMotor();
    }
}