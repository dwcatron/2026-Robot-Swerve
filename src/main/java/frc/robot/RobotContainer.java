package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets; 
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser; 
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import java.util.Map; 

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.generated.constants; 
import frc.robot.subsystems.*;
import frc.robot.commands.*;

public class RobotContainer {
    /* Drive Constants */
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(1.5).in(RadiansPerSecond);

    /* Swerve Requests */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.03).withRotationalDeadband(MaxAngularRate * 0.03)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final Telemetry logger = new Telemetry(MaxSpeed);

    /* Controllers */
    private final CommandXboxController m_driverController = new CommandXboxController(0);
    private final CommandXboxController m_operatorController = new CommandXboxController(1);

    /* Subsystems */
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final Intake m_intake = new Intake();
    private final Shooter m_shooter = new Shooter();
    private final Hopper m_hopper = new Hopper();
    private final Climber m_climber = new Climber();
    private final Turret m_turret = new Turret();
    private final Hood m_hood = new Hood();
    private final Indexer m_indexer = new Indexer();

    /* --- THE AUTO CHOOSER --- */
    private final SendableChooser<Command> autoChooser;

    /* Restored Command Groups */
    SequentialCommandGroup Start_Match = new SequentialCommandGroup(
        new HopperOut(m_hopper, 12));

    SequentialCommandGroup Shoot = new SequentialCommandGroup(
        new Shooter_default(m_shooter, 4500));

    SequentialCommandGroup Climb = new SequentialCommandGroup(
        new L_Three_Climb(m_climber, 22.0),
        new WaitCommand(0.5),
        new L_Three_Climb(m_climber, -5.0));

   public RobotContainer() {
        setupDashboard();
        configureBindings();
        SmartDashboard.putData("Run System Check", systemCheckCommand());
        
        // This must be called AFTER configureBindings() so it knows about your NamedCommands!
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Routine", autoChooser);
    }

    private double getLimelightDistance() {
        // UPDATED NAME HERE
        double ty = NetworkTableInstance.getDefault().getTable("limelight_tauret").getEntry("ty").getDouble(0);
        return (2.6 - 0.5) / Math.tan(Math.toRadians(30.0 + ty));
    }

    private double getClimberTagOffset() {
        return NetworkTableInstance.getDefault().getTable("limelight-climber").getEntry("tx").getDouble(0);
    }
    
    public void periodic() {} 

    private void configureBindings() {
        /* --- PATHPLANNER / NAMED COMMANDS --- */
        NamedCommands.registerCommand("START", Start_Match);
        NamedCommands.registerCommand("Shoot", Shoot);
        NamedCommands.registerCommand("Climb", Climb);
        
        NamedCommands.registerCommand("AutoPrep", VisionAimAndReady.getCommand(m_shooter, m_turret, m_hood, this::getLimelightDistance));
        NamedCommands.registerCommand("AlignToTower", 
            drivetrain.applyRequest(() -> drive.withRotationalRate(getClimberTagOffset() * -0.05))
            .until(() -> Math.abs(getClimberTagOffset()) < 1.0).withTimeout(2.0));

        /* --- DRIVER CONTROLS (Port 0) --- */
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-m_driverController.getLeftY() * MaxSpeed)
                     .withVelocityY(-m_driverController.getLeftX() * MaxSpeed)
                     .withRotationalRate(-m_driverController.getRightX() * MaxAngularRate)
            )
        );

        m_driverController.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        m_driverController.rightBumper().whileTrue(Commands.parallel(
            new RunIntake(m_intake, -0.9, 0.9),
            Commands.startEnd(() -> m_indexer.setPercent(-0.6), m_indexer::stop, m_indexer)
        ));

        m_driverController.back().whileTrue(
        new StartEndCommand(
            () -> m_hood.setMainMotorSpeed(0.6),  
            () -> m_hood.setMainMotorSpeed(0.0),  
            m_hood                       
        ));

        m_driverController.start().whileTrue(
        new StartEndCommand(
            () -> m_hood.setMainMotorSpeed(-0.6), 
            () -> m_hood.setMainMotorSpeed(0.0),  
            m_hood                       
        ));

        m_driverController.start().and(m_driverController.b()).whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-m_driverController.getLeftY(), -m_driverController.getLeftX()))
        ));

        m_driverController.back().and(m_driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        m_driverController.back().and(m_driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        m_driverController.start().and(m_driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        m_driverController.start().and(m_driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        /* --- OPERATOR CONTROLS (Port 1) --- */
        m_operatorController.start().toggleOnTrue(
        VisionAimAndReady.getCommand(m_shooter, m_turret, m_hood, this::getLimelightDistance)
        );
        m_operatorController.rightTrigger().whileTrue(
            new FireFuel(m_intake, m_indexer, m_shooter, m_turret, m_operatorController, this::getLimelightDistance)
        );
        m_operatorController.rightBumper().whileTrue(
            new Shooter_test(m_shooter)
        );
      
       m_operatorController.leftBumper().whileTrue(
            new TurretTrackTarget(m_turret, () -> getLimelightAngle(), this) 
        );

        // Move Up while holding POV Up
        m_operatorController.povUp().whileTrue(
            m_climber.startEnd(
                () -> m_climber.moveManual(0.2), // Start action
                () -> m_climber.stopMotor()      // End action
            )
        );

        // Move Down while holding POV Down
        m_operatorController.povDown().whileTrue(
            m_climber.startEnd(
                () -> m_climber.moveManual(-0.2), // Start action
                () -> m_climber.stopMotor()       // End action
            )
        );

        m_operatorController.y().onTrue(new SequentialCommandGroup(
            new L_Three_Climb(m_climber, 22.0),
            new WaitCommand(0.5),
            new L_Three_Climb(m_climber, 0.0),
            new WaitCommand(0.5),
            new L_Three_Climb(m_climber, 22.0)
        ));

       m_operatorController.x().whileTrue(Commands.parallel(
            new RunIntakeOut(m_intake, 0.9, -0.9),
            new RunIndexer(m_indexer, 0.6) 
        ));

        m_operatorController.a().whileTrue(Commands.parallel(
            new RunIntake(m_intake, -0.9, 0.9),
            Commands.startEnd(() -> m_indexer.setPercent(-0.6), m_indexer::stop, m_indexer)
        ));

        /* --- SYSTEM UTILITIES --- */
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        /* --- BACKGROUND HEALTH MONITOR --- */
        Commands.run(() -> {
            double amps = m_intake.getFrontCurrent(); 
            SmartDashboard.putNumber("Intake/Front Amps", amps);
            SmartDashboard.putBoolean("Intake/JAMMED", amps > 35.0);
            
            if (amps > 35.0) {
                m_driverController.getHID().setRumble(RumbleType.kBothRumble, 0.5);
            } else {
                m_driverController.getHID().setRumble(RumbleType.kBothRumble, 0);
            }
        }, m_intake).ignoringDisable(true).schedule(); 
        
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    private void setupDashboard() {
        ShuffleboardTab driverTab = Shuffleboard.getTab("Driver");

        driverTab.addBoolean("READY TO FIRE", () -> {
            // UPDATED NAMES HERE
            boolean hasTarget = NetworkTableInstance.getDefault().getTable("limelight_tauret").getEntry("tv").getDouble(0) == 1.0;
            boolean turretLocked = Math.abs(NetworkTableInstance.getDefault().getTable("limelight_tauret").getEntry("tx").getDouble(0)) < 2.0;
            boolean shooterReady = m_shooter.isAtSpeed(100);
            return hasTarget && turretLocked && shooterReady;
        })
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withPosition(0, 0)
        .withSize(3, 3)
        .withProperties(Map.of("Color when true", "Lime", "Color when false", "Red"));

        // FIX: Using an explicit lambda here stops the (String, Shooter) compiler crash!
        driverTab.addDouble("Shooter RPM", () -> m_shooter.getCurrentRPM())
        .withWidget(BuiltInWidgets.kDial)
        .withPosition(3, 0)
        .withSize(2, 2)
        .withProperties(Map.of("min", 0, "max", 6000));

        // FIX: Explicit lambda prevents the same issue here
        driverTab.addDouble("Turret Angle", () -> m_turret.getAngle())
        .withWidget(BuiltInWidgets.kDial)
        .withPosition(5, 0)
        .withSize(2, 2)
        .withProperties(Map.of("min", constants.kTurretMinAngle, "max", constants.kTurretMaxAngle));

        driverTab.addDouble("Target Distance", () -> getLimelightDistance())
        .withWidget(BuiltInWidgets.kNumberBar)
        .withPosition(3, 2)
        .withSize(4, 1)
        .withProperties(Map.of("min", 0, "max", 20)); 

        // --- NEW: LIMELIGHT 4 (FUEL) SHUFFLEBOARD WIDGETS ---
        
        // 1. A box that turns Green when the LL4 sees fuel, Red when it doesn't
        driverTab.addBoolean("Fuel Spotted", () -> 
            NetworkTableInstance.getDefault().getTable("limelight-fuel").getEntry("tv").getDouble(0) == 1.0)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withPosition(7, 0) 
        .withSize(2, 2)
        .withProperties(Map.of("Color when true", "Lime", "Color when false", "Red"));

        // 2. A bar showing how far left or right the fuel is from the center
        driverTab.addDouble("Fuel X Offset", () -> 
            NetworkTableInstance.getDefault().getTable("limelight-fuel").getEntry("tx").getDouble(0.0))
        .withWidget(BuiltInWidgets.kNumberBar)
        .withPosition(7, 2) 
        .withSize(2, 1)
        .withProperties(Map.of("min", -30, "max", 30)); 
    }

    public double getLimelightAngle() {
        // UPDATED NAME HERE
        return m_turret.getAngle() + NetworkTableInstance.getDefault().getTable("limelight_tauret").getEntry("tx").getDouble(0.0);
    }

    public void setLimelightPipeline(int pipeline) {
        // UPDATED NAME HERE
        NetworkTableInstance.getDefault().getTable("limelight_tauret").getEntry("pipeline").setNumber(pipeline);
    }

    public Command systemCheckCommand() {
        return Commands.sequence(
            Commands.print("Starting System Check..."),
            m_intake.runOnce(() -> m_intake.setSpeed(0.2, 0.2)).withTimeout(0.5),
            m_intake.runOnce(() -> m_intake.stop()),
            m_indexer.runOnce(() -> m_indexer.setPercent(0.3)).withTimeout(0.5),
            m_indexer.runOnce(() -> m_indexer.stop()),
            m_turret.runOnce(() -> m_turret.setSpeed(0.1)).withTimeout(0.3),
            m_turret.runOnce(() -> m_turret.stop()),
            m_shooter.runOnce(() -> m_shooter.setRPM(500)).withTimeout(1.0),
            m_shooter.runOnce(() -> m_shooter.stop()),
            Commands.print("System Check Complete!")
        ).ignoringDisable(false); 
    }
   
    /* --- USE THE CHOOSER FOR AUTO --- */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}