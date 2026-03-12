package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.generated.constants;

public class Shooter extends SubsystemBase {

    // --- SHOOTER HARDWARE ---
    private final TalonFX m_leftMotor = new TalonFX(constants.kShooter_LeftCanId);
    private final TalonFX m_rightMotor = new TalonFX(constants.kShooter_RightCanId);
    private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0);

    // --- STATE VARIABLES ---
    private double m_targetRPM = 0;
    private final InterpolatingDoubleTreeMap m_rpmMap = new InterpolatingDoubleTreeMap();

    // --- PHYSICS MEASUREMENTS (in Meters and Radians) ---
    private static final double GRAVITY = 9.81;
    private static final double SHOOTER_ANGLE_RAD = Math.toRadians(45.0); // Replace with your angle
    private static final double WHEEL_RADIUS_METERS = 0.0508; // 2-inch wheel
    
    // Heights
    private static final double SHOOTER_HEIGHT = 0.6858; // 27 inches
    private static final double HUB_HEIGHT = 1.26; 
    private static final double APRIL_TAG_HEIGHT = 0.40; // Replace with actual tag height
    
    // Delta Y values
    private static final double DELTA_Y_HUB = HUB_HEIGHT - SHOOTER_HEIGHT; // What we shoot at
    private static final double DELTA_Y_TAG = APRIL_TAG_HEIGHT - SHOOTER_HEIGHT; // What we measure with
    private static final double EFFICIENCY_MULTIPLIER = 1.2; // Tune this for real-world physics

    public Shooter() {
        // --- MOTOR CONFIGURATION ---
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.StatorCurrentLimit = 80.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        config.Slot0.kP = constants.kShooter_kP;
        config.Slot0.kV = constants.kShooter_kV;

        m_leftMotor.getConfigurator().apply(config);

        m_rightMotor.setControl(
                new Follower(
                        m_leftMotor.getDeviceID(),
                        MotorAlignmentValue.Opposed));

        // --- Distance → RPM map (Manual Overrides) ---
        /* m_rpmMap.put(1.5, 2800.0);
        m_rpmMap.put(2.0, 3200.0);
        m_rpmMap.put(2.5, 3600.0);
        m_rpmMap.put(3.0, 4100.0);
        m_rpmMap.put(3.5, 4600.0);
        */
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
    public void setRPMFromDistanceMap(double distanceMeters) {
        distanceMeters = MathUtil.clamp(distanceMeters, 1.2, 4.0);
        double rpm = m_rpmMap.get(distanceMeters);
        setRPM(rpm);
    }

    /** Sets RPM using real-time physics calculations */
    public void setRPMFromPhysics(double hypotenuseToTag) {
        double calculatedRPM = calculateHubRPM(hypotenuseToTag);
        setRPM(calculatedRPM);
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
        // m_turretMotor.set(0); // Uncomment if you want stop() to halt the turret too
    }

    // ==========================================================
    // PHYSICS & VISION LOGIC
    // ==========================================================

    /**
     * Calculates the RPM needed to hit the 1.26m Hub based on the AprilTag's hypotenuse.
     */
    public double calculateHubRPM(double hypotenuseToTag) {
        // 1. Find flat horizontal distance (x) using the AprilTag
        double radicand = Math.pow(hypotenuseToTag, 2) - Math.pow(DELTA_Y_TAG, 2);
        double horizontalDistance = Math.sqrt(Math.max(0, radicand));

        // 2. Calculate exit velocity (v) aiming at the HUB
        double numerator = GRAVITY * Math.pow(horizontalDistance, 2);
        double cosTheta = Math.cos(SHOOTER_ANGLE_RAD);
        double tanTheta = Math.tan(SHOOTER_ANGLE_RAD);
        
        double denominator = 2 * Math.pow(cosTheta, 2) * ((horizontalDistance * tanTheta) - DELTA_Y_HUB);
        
        if (denominator <= 0) return 0.0; // Mathematically impossible to hit

        double requiredVelocity = Math.sqrt(numerator / denominator) * EFFICIENCY_MULTIPLIER;
        
        // 3. Convert velocity to RPM
        double wheelSurfaceSpeed = requiredVelocity * 2.0; 
        return (wheelSurfaceSpeed * 60.0) / (2 * Math.PI * WHEEL_RADIUS_METERS);
    }

    
}