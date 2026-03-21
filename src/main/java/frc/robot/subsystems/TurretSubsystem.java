package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretSubsystem extends SubsystemBase {

    // ===================== HARDWARE =====================
 private final SparkMax motor =
   
            new SparkMax(10, MotorType.kBrushless);  // CHANGE CAN ID

    private final CANcoder encoder =
            new CANcoder(20);  // CHANGE CAN ID

    private final NetworkTable limelight =
            NetworkTableInstance.getDefault().getTable("limelight");

    // ===================== CONTROL =====================

    private final PIDController pid =
            new PIDController(6.0, 0.0, 0.25);  // STARTING GAINS — TUNE

    private final TrapezoidProfile.Constraints constraints =
            new TrapezoidProfile.Constraints(
                    6.0,   // max velocity (rad/sec)
                    15.0   // max acceleration (rad/sec^2)
            );

    private TrapezoidProfile.State profileSetpoint =
            new TrapezoidProfile.State(0, 0);

    private double targetAngleRad = 0;
    private boolean closedLoop = false;
    private double lastTime = Timer.getFPGATimestamp();

    // ===================== SOFT LIMITS =====================

    private static final double MIN_ANGLE_RAD =
            Units.degreesToRadians(-120);  // CHANGE FOR YOUR MECHANISM

    private static final double MAX_ANGLE_RAD =
            Units.degreesToRadians(120);   // CHANGE FOR YOUR MECHANISM

    // ===================== CAMERA OFFSET =====================

    // 6 inches left of turret center
    private static final double CAMERA_OFFSET_METERS = 0.1524;

    // ===================== PUBLIC DISTANCE =====================

    public double distanceMeters = 0.0;

    // ===========================================================

    public TurretSubsystem() {

        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(25)
              .idleMode(IdleMode.kBrake);

        motor.configure(
                config,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters
        );

        pid.setTolerance(Units.degreesToRadians(1.0));

        limelight.getEntry("pipeline").setNumber(0);
    }

    // ===================== PERIODIC =====================

    @Override
    public void periodic() {

        if (!closedLoop) return;

        double now = Timer.getFPGATimestamp();
        double dt = now - lastTime;
        lastTime = now;

        TrapezoidProfile profile =
                new TrapezoidProfile(constraints);

        TrapezoidProfile.State goal =
                new TrapezoidProfile.State(targetAngleRad, 0);

        profileSetpoint =
                profile.calculate(dt, profileSetpoint, goal);

        double currentAngle = getAngleRadians();

        double output =
                pid.calculate(currentAngle, profileSetpoint.position);

        output = MathUtil.clamp(output, -0.6, 0.6);

        motor.set(output);
    }

    // ===================== ANGLE CONTROL =====================

    public void setAngleRadians(double radians) {

        radians = MathUtil.clamp(
                radians,
                MIN_ANGLE_RAD,
                MAX_ANGLE_RAD
        );

        targetAngleRad = radians;
        closedLoop = true;
    }

    public void stop() {
        closedLoop = false;
        motor.stopMotor();
        profileSetpoint =
                new TrapezoidProfile.State(getAngleRadians(), 0);
    }

    // ===================== SENSOR =====================

    public double getAngleRadians() {

        // CANcoder returns rotations
        return Units.rotationsToRadians(
                encoder.getAbsolutePosition()
                       .refresh()
                       .getValueAsDouble()
        );
    }

    // ===================== VISION TRACKING =====================

    public void trackAprilTag() {

        double tv =
                limelight.getEntry("tv").getDouble(0);

        if (tv < 1.0) return;

        double txDeg =
                limelight.getEntry("tx").getDouble(0);

        double txRad =
                Units.degreesToRadians(txDeg);

        double[] pose =
                limelight.getEntry("targetpose_robotspace")
                         .getDoubleArray(new double[6]);

        double forward = pose[2];
        double sideways = pose[0];

        distanceMeters =
                Math.sqrt(forward * forward +
                          sideways * sideways);

        if (distanceMeters < 0.05) return;

        // Offset correction for camera mounted 6in left
        double correctedAngle =
                Math.atan(
                        Math.tan(txRad) +
                        (CAMERA_OFFSET_METERS /
                                distanceMeters)
                );

        double newTarget =
                getAngleRadians() + correctedAngle;

        setAngleRadians(newTarget);
    }
}