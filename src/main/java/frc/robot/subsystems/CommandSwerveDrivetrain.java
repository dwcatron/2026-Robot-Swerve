package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.004;
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    private boolean m_hasAppliedOperatorPerspective = false;
    private final Field2d m_field = new Field2d();
    // FIX 1: Changed from ApplyChassisSpeeds to ApplyRobotSpeeds (CTRE 2025/2026 Standard)
    private final SwerveRequest.ApplyRobotSpeeds AutoRequest = new SwerveRequest.ApplyRobotSpeeds();
    
    /* --- RESTORED SYSID ROUTINES FROM ORIGINAL FILE --- */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(null, Volts.of(4), null, state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
        new SysIdRoutine.Mechanism(output -> setControl(m_translationCharacterization.withVolts(output)), null, this)
    );

    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(null, Volts.of(7), null, state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
        new SysIdRoutine.Mechanism(volts -> setControl(m_steerCharacterization.withVolts(volts)), null, this)
    );

    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(Volts.of(Math.PI / 6).per(Second), Volts.of(Math.PI), null, state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
        new SysIdRoutine.Mechanism(output -> {
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            }, null, this)
    );

    /* Default routine to apply is translation */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    /* --- CONSTRUCTORS --- */
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) { startSimThread(); }
        configurePathPlanner(); 
        SmartDashboard.putData("Field", m_field);
    }

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants drivetrainConstants, double odometryUpdateFrequency, SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) { startSimThread(); }
        configurePathPlanner(); 
        SmartDashboard.putData("Field", m_field);
    }

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants drivetrainConstants, double odometryUpdateFrequency, Matrix<N3, N1> odometryStandardDeviation, Matrix<N3, N1> visionStandardDeviation, SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        if (Utils.isSimulation()) { startSimThread(); }
        configurePathPlanner(); 
        SmartDashboard.putData("Field", m_field);
    }

    /* --- PATHPLANNER CONFIGURATION --- */
    private void configurePathPlanner() {
        try {
            // Loads your robot's physical settings directly from the PathPlanner GUI
            RobotConfig config = RobotConfig.fromGUISettings();

            AutoBuilder.configure(
                () -> this.getState().Pose, // Method to get the robot's current position
                this::resetPose, // FIX 2: Used resetPose instead of seedFieldCentric
                () -> this.getState().Speeds, // Method to get current chassis speeds
                (speeds, feedforwards) -> this.setControl(AutoRequest.withSpeeds(speeds)), // Drives the robot
                new PPHolonomicDriveController(
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID (Moving X/Y)
                    new PIDConstants(10.0, 0.0, 0.0)  // Rotation PID (Spinning)
                ),
                config, // Passes in the GUI configuration
                () -> {
                    // Automatically mirrors paths if you are on the Red Alliance
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Require this subsystem so teleop doesn't fight auto
            );
        } catch (Exception e) {
            // FIX 3: Corrected the reportError syntax
            DriverStation.reportError("Failed to load PathPlanner config. Check your GUI settings!", false);
            e.printStackTrace();
        }
    }

    /* --- VISION LOGIC --- */
   public void updatePoseFromLimelight(String tableName) {
    double[] botpose = NetworkTableInstance.getDefault().getTable(tableName).getEntry("botpose_wpiblue").getDoubleArray(new double[0]);
    
    if (botpose.length >= 8 && botpose[7] > 0) {
        Pose2d visionPose = new Pose2d(botpose[0], botpose[1], Rotation2d.fromDegrees(botpose[5]));
        double timestamp = Timer.getFPGATimestamp() - (botpose[6] / 1000.0);
        double xyStdDev = botpose[9] > 4.0 ? 0.2 : 0.8; 
        double thetaStdDev = 999.0; 
        addVisionMeasurement(visionPose, timestamp, VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev));
    }
}

    @Override
    public void periodic() {
        updatePoseFromLimelight("limelight-fuel");

        Pose2d pose = getState().Pose;
        SmartDashboard.putNumber("Robot X", pose.getX());
        SmartDashboard.putNumber("Robot Y", pose.getY());
        
        // YOU MISSED THIS LINE RIGHT HERE:
        m_field.setRobotPose(pose);

        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(allianceColor == Alliance.Red ? Rotation2d.k180deg : Rotation2d.kZero);
                m_hasAppliedOperatorPerspective = true;
            });
        }
    }

    /* --- SYSID METHODS --- */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    /* --- UTILITIES --- */
    public Command applyRequest(Supplier<SwerveRequest> request) {
        return run(() -> this.setControl(request.get()));
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            updateSimState(currentTime - m_lastSimTime, RobotController.getBatteryVoltage());
            m_lastSimTime = currentTime;
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
    }

    @Override
    public Optional<Pose2d> samplePoseAt(double timestampSeconds) {
        return super.samplePoseAt(Utils.fpgaToCurrentTime(timestampSeconds));
    }
}