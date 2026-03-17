package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.constants;
/**
 * Unified turret subsystem with three operating modes:
 *   1. Manual speed control
 *   2. Closed-loop angle set-point (PID via CANcoder feedback)
 *   3. Vision tracking via Limelight (PD controller targeting an AprilTag)
 */
public class Turret extends SubsystemBase {
    // ── Operating modes ──────────────────────────────────────────────
    private enum TurretMode { MANUAL, CLOSED_LOOP_ANGLE, VISION_TRACKING }
    // ── Hardware ─────────────────────────────────────────────────────
    private final SparkMax m_motor =
            new SparkMax(constants.kTurretCanId, MotorType.kBrushless);
    private final CANcoder m_encoder =
            new CANcoder(constants.kTurretEncoderCanId);
    // ── Closed-loop angle PID ────────────────────────────────────────
    private final PIDController m_pid =
            new PIDController(constants.kTurretP,
                              constants.kTurretI,
                              constants.kTurretD);
    // ── Vision-tracking PD gains (ported from TurretTemporary) ───────
    private double m_trackingKp = 0.02;
    private double m_trackingKd = 0.001;
    private double m_trackingLastError = 0.0;
    private double m_trackingGoalX = 0.0;          // desired tx offset
    private double m_trackingAngleTolerance = 0.2;  // degrees
    // ── Limelight NetworkTables ──────────────────────────────────────
    private final NetworkTableEntry m_tv;   // 1 = target visible
    private final NetworkTableEntry m_tx;   // horizontal offset (deg)
    private final NetworkTableEntry m_tid;  // detected AprilTag ID
    // ── Timing for derivative term ──────────────────────────────────
    private double m_lastTimestamp = Timer.getFPGATimestamp();
    // ── State ────────────────────────────────────────────────────────
    private double m_targetAngle = constants.kTurretCenterAngle;
    private TurretMode m_mode = TurretMode.MANUAL;
    private int m_targetTagId = 9;  // default tag to track
    // ═════════════════════════════════════════════════════════════════
    //  Constructor
    // ═════════════════════════════════════════════════════════════════
    public Turret() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(25)
              .idleMode(IdleMode.kBrake);
        m_motor.configure(config,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        m_pid.setTolerance(1.0);
        // Limelight NetworkTables setup
        NetworkTable limelight =
                NetworkTableInstance.getDefault().getTable("limelight");
        m_tv  = limelight.getEntry("tv");
        m_tx  = limelight.getEntry("tx");
        m_tid = limelight.getEntry("tid");
    }
    // ═════════════════════════════════════════════════════════════════
    //  Periodic (runs every 20 ms)
    // ═════════════════════════════════════════════════════════════════
    @Override
    public void periodic() {
        double currentAngle = getAngle();
        // ── Emergency hard stop if past absolute limits ──
        if (currentAngle > constants.kTurretMaxAngle + 2 ||
            currentAngle < constants.kTurretMinAngle - 2) {
            m_motor.stopMotor();
            m_mode = TurretMode.MANUAL;
            return;
        }
        switch (m_mode) {
            case CLOSED_LOOP_ANGLE:
                runClosedLoopAngle(currentAngle);
                break;
            case VISION_TRACKING:
                runVisionTracking(currentAngle);
                break;
            case MANUAL:
            default:
                // motor is set directly via setSpeed(); nothing to do here
                break;
        }
    }
    // ── Closed-loop angle control ────────────────────────────────────
    private void runClosedLoopAngle(double currentAngle) {
        double output = m_pid.calculate(currentAngle, m_targetAngle);
        output = MathUtil.clamp(output,
                -constants.kTurretMaxOutput,
                 constants.kTurretMaxOutput);
        m_motor.set(output);
    }
    // ── Vision tracking (ported from TurretTemporary) ────────────────
    private void runVisionTracking(double currentAngle) {
        double now = Timer.getFPGATimestamp();
        double deltaTime = now - m_lastTimestamp;
        m_lastTimestamp = now;
        boolean hasTarget = m_tv.getDouble(0) >= 1.0;
        int detectedId    = (int) m_tid.getDouble(-1);
        // *** BUG FIX: original code was (curID == null && curID.id == 9)
        //     which would NullPointerException.  Correct logic:
        //     stop if there is NO target OR the wrong tag is detected. ***
        if (!hasTarget || detectedId != m_targetTagId) {
            m_motor.set(0);
            m_trackingLastError = 0;
            return;
        }
        // tx = horizontal offset from Limelight crosshair (degrees)
        double tx = m_tx.getDouble(0);
        // PD controller  (goalX is typically 0 → center the tag)
        double error = m_trackingGoalX - tx;
        double pTerm = error * m_trackingKp;
        double dTerm = 0;
        if (deltaTime > 0) {
            dTerm = ((error - m_trackingLastError) / deltaTime) * m_trackingKd;
        }
        // *** BUG FIX: `power` was declared `final` in TurretTemporary,
        //     which prevented reassignment.  Now a local variable. ***
        double power;
        if (Math.abs(error) < m_trackingAngleTolerance) {
            power = 0;
        } else {
            power = MathUtil.clamp(pTerm + dTerm,
                    -constants.kTurretMaxOutput,
                     constants.kTurretMaxOutput);
        }
        // Safety encoder check — respect soft limits while tracking
        if (currentAngle > constants.kTurretMaxAngle ||
            currentAngle < constants.kTurretMinAngle) {
            m_motor.stopMotor();
            m_trackingLastError = 0;
            return;
        }
        m_motor.set(power);
        m_trackingLastError = error;
    }
    // ═════════════════════════════════════════════════════════════════
    //  Public API
    // ═════════════════════════════════════════════════════════════════
    /** Closed-loop: drive the turret to an absolute angle (degrees). */
    public void setAngle(double degrees) {
        double normalized = normalizeAngle(degrees);
        if (!isWithinLimits(normalized)) {
            normalized = closestLimit(normalized);
        }
        m_targetAngle = normalized;
        m_mode = TurretMode.CLOSED_LOOP_ANGLE;
    }
    /** Manual speed mode (–1 … +1). */
    public void setSpeed(double speed) {
        m_mode = TurretMode.MANUAL;
        speed = MathUtil.clamp(speed,
                -constants.kTurretMaxOutput,
                 constants.kTurretMaxOutput);
        m_motor.set(speed);
    }
    /** Start vision tracking for a specific AprilTag ID via Limelight. */
    public void startTracking(int tagId) {
        m_targetTagId = tagId;
        m_trackingLastError = 0;
        m_lastTimestamp = Timer.getFPGATimestamp();
        m_mode = TurretMode.VISION_TRACKING;
    }
    /** Start vision tracking using the currently configured tag ID. */
    public void startTracking() {
        startTracking(m_targetTagId);
    }
    /** Stop all turret motion. */
    public void stop() {
        m_mode = TurretMode.MANUAL;
        m_motor.stopMotor();
    }
    /** Drive turret to its center position. */
    public void centerTurret() {
        setAngle(constants.kTurretCenterAngle);
    }
    // ── Tracking-gain getters / setters ──────────────────────────────
    public void setTrackingKp(double kp)          { m_trackingKp = kp; }
    public double getTrackingKp()                 { return m_trackingKp; }
    // *** BUG FIX: original TurretTemporary had `setkD()` as the getter
    //     name — renamed to `getTrackingKd()`. ***
    public void setTrackingKd(double kd)          { m_trackingKd = kd; }
    public double getTrackingKd()                 { return m_trackingKd; }
    public void setTrackingGoalX(double goalX)    { m_trackingGoalX = goalX; }
    public void setTrackingTolerance(double tol)  { m_trackingAngleTolerance = tol; }
    public void setTargetTagId(int id)            { m_targetTagId = id; }
    // ── Sensor helpers ───────────────────────────────────────────────
    /** Current turret angle from the CANcoder (degrees). */
    public double getAngle() {
        return (m_encoder.getAbsolutePosition()
                .refresh()
                .getValueAsDouble() * 360.0)
                - constants.kTurretOffset;
    }
    /** True when the turret is within the lock tolerance of the target. */
    public boolean isLocked(double targetAngle) {
        return Math.abs(getAngle() - targetAngle)
                < constants.kTurretLockToleranceDeg;
    }
    /** True when the subsystem is actively tracking via Limelight. */
    public boolean isTracking() {
        return m_mode == TurretMode.VISION_TRACKING;
    }
    /** True if the turret is currently tracking and aimed perfectly at the target */
    public boolean isOnTarget() {
        return m_mode == TurretMode.VISION_TRACKING
            && hasVisionTarget()
            && Math.abs(m_trackingLastError) < m_trackingAngleTolerance;
    }
    /** True when Limelight sees any valid target. */
    public boolean hasVisionTarget() {
        return m_tv.getDouble(0) >= 1.0;
    }
    // ── Angle-math helpers ───────────────────────────────────────────
    private double normalizeAngle(double angle) {
        angle %= 360.0;
        if (angle < 0) angle += 360.0;
        return angle;
    }
    private boolean isWithinLimits(double angle) {
        double min = constants.kTurretMinAngle;
        double max = constants.kTurretMaxAngle;
        if (min > max) {
            return angle >= min || angle <= max;   // wrapped range
        }
        return angle >= min && angle <= max;
    }
    private double closestLimit(double angle) {
        double min = constants.kTurretMinAngle;
        double max = constants.kTurretMaxAngle;
        double distToMin = angularDistance(angle, min);
        double distToMax = angularDistance(angle, max);
        return distToMin < distToMax ? min : max;
    }
    private double angularDistance(double a, double b) {
        double diff = Math.abs(a - b) % 360;
        return diff > 180 ? 360 - diff : diff;
    }
}
