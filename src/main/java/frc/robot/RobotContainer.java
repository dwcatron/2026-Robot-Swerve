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
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
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
        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.07)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final SwerveRequest.FieldCentric facingAim = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withDeadband(MaxSpeed * 0.1);


    /* Controllers */
    private final CommandXboxController m_operatorController = new CommandXboxController(0);
    private final CommandXboxController m_driverController = new CommandXboxController(1);
    private final CommandXboxController m_driverController1 = new CommandXboxController(2);


    /* Subsystems */
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final Intake m_intake = new Intake();
    private final Shooter m_shooter = new Shooter();
    private final Hopper m_hopper = new Hopper();
    private final Turret m_turret = new Turret();
    private final Hood m_hood = new Hood();


    /* --- THE AUTO CHOOSER --- */
    private final SendableChooser<Command> autoChooser;


    /* --- DASHBOARD ENTRIES --- */
    private GenericEntry m_smartShootToggle;


    /**
     * Calculates the rotation speed needed to align the robot chassis with the Limelight target.
     */
    private double getLimelightRotationRate() {
        var table = NetworkTableInstance.getDefault().getTable("limelight_turret");
        double tv = table.getEntry("tv").getDouble(0); // 1.0 if target is found
        double tx = table.getEntry("tx").getDouble(0); // Degrees from center (-29.8 to 29.8)


        // If no target is seen, don't spin the robot wildly; return 0.
        if (tv < 1.0) return 0.0;


        /* * TUNING: 0.05 is your 'P' gain.
         * If the robot is too slow to aim, increase this (e.g., 0.07).
         * If it overshoots or shakes, decrease this (e.g., 0.03).
         */
        double rotationOutput = tx * -0.05 * MaxAngularRate;
        return rotationOutput;
    }


    /* Restored Command Groups */
    /* Restored Command Groups */
    SequentialCommandGroup Start_Match = new SequentialCommandGroup();


    SequentialCommandGroup Shoot = new SequentialCommandGroup(
        new FireFuel(m_shooter, m_turret, m_hood, m_intake, this::getLimelightDistance)
            .withTimeout(2.5)
    );


 
    public RobotContainer() {
        setupDashboard();
        configureBindings();
        //drivetrain.seedFieldCentric();


        SmartDashboard.putData("Run System Check", systemCheckCommand());


        // This must be called AFTER configureBindings() so it knows about your NamedCommands!
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Routine", autoChooser);
    }


    private double getLimelightTX() {
        return NetworkTableInstance.getDefault().getTable("limelight_turret").getEntry("tx").getDouble(0);
    }


    private double getLimelightDistance() {
        // UPDATED NAME HERE
        double ty = NetworkTableInstance.getDefault().getTable("limelight_turret").getEntry("ty").getDouble(0);
        return (2.6 - 0.5) / Math.tan(Math.toRadians(30.0 + ty));
    }


   
    public void periodic() {
        // Read the smart shooting toggle from Shuffleboard each loop
        constants.kEnableSmartShooting = m_smartShootToggle.getBoolean(true);
    }


    private void configureBindings() {
        /* --- PATHPLANNER / NAMED COMMANDS --- */
        NamedCommands.registerCommand("START", Start_Match);
        NamedCommands.registerCommand("Shoot", Shoot);


        /* --- DRIVER CONTROLS (Port 0) --- */
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-m_driverController.getLeftY() * MaxSpeed)
                     .withVelocityY(-m_driverController.getLeftX() * MaxSpeed)
                     .withRotationalRate(-m_driverController1.getLeftX() * MaxAngularRate)
            )
        );


        m_driverController1.povUp().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        m_driverController.rightBumper().whileTrue(
            new RunIntake(m_intake, -0.9, 0.9)
        );


        m_driverController.start().and(m_driverController.b()).whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-m_driverController.getLeftY(), -m_driverController.getLeftX()))
        ));


        m_driverController.back().and(m_driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        m_driverController.back().and(m_driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        m_driverController.start().and(m_driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        m_driverController.start().and(m_driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));


        /* --- OPERATOR CONTROLS (Port 1) --- */
        //  m_operatorController.start().toggleOnTrue(
        //  VisionAimAndReady.getCommand(m_shooter, m_turret, m_hood, this::getLimelightDistance)
        //  );


        m_operatorController.back().whileTrue(m_hood.runEnd(() -> m_hood.setMainMotorSpeed(0.2d), () -> m_hood.stop()));
        m_operatorController.start().whileTrue(m_hood.runEnd(() -> m_hood.setMainMotorSpeed(-0.2d), () -> m_hood.stop()));


        // Inside configureBindings()
        m_driverController.leftBumper().whileTrue(drivetrain.applyRequest(() ->
            facingAim.withVelocityX(-m_driverController.getLeftY() * MaxSpeed)
                     .withVelocityY(-m_driverController.getLeftX() * MaxSpeed)
                     .withRotationalRate(getLimelightRotationRate())
        ));
       
        /*m_operatorController.leftBumper().whileTrue(
            new TurretTrackTarget(m_turret, () -> getLimelightAngle(), this)
        );*/


<<<<<<< HEAD
      

       
=======
        // Move Up while holding POV Up
       m_operatorController.povUp().whileTrue(
    new TrackTargetCommand(m_turret)
);
>>>>>>> d987f2f7cdc6a797e3c05f278709f98d5f65c349





        m_operatorController.x().whileTrue(
            new RunIntakeOut(m_intake, 0.9, -0.9)
        );


        m_operatorController.a().whileTrue(
            new RunIntake(m_intake, -0.9, 0.9)
        );


        m_operatorController.b().whileTrue(
            new RunHopperIntake(m_intake, 0.75)
        );


        /*m_operatorController.iftherobotdoesn'tdrive().whileTrue(
            new CrashOut(m_crashingOut, 100)
        );*/
        // Operator Right Trigger: Unified Smart Fire
        // Operator Right Trigger: Unified Smart Fire
        // Operator Right Trigger: Unified Smart Fire
        m_operatorController.rightTrigger().whileTrue(
            new FireFuel(m_shooter, m_turret, m_hood, m_intake, this::getLimelightDistance)
        );


        // --- REAR INTAKE DEPLOYMENT ---
        // Hold Left Bumper to push the hopper out at 20% speed
        m_operatorController.leftBumper().whileTrue(new EjectHopper(m_hopper, 0.20));


        // Hold Right Bumper to pull the hopper back in at 20% speed
        m_operatorController.rightBumper().whileTrue(new EjectHopper(m_hopper, -0.20));
       
        // Operator Left Trigger: Field-Centric Auto-Aiming Pass
        m_operatorController.leftTrigger().whileTrue(
            new PassFuel(
                m_shooter,
                m_turret,
                m_hood,
                m_intake,
                () -> drivetrain.getState().Pose // <--- This feeds the live field coordinates!
            )
        );
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
       
        // FIX: Using an explicit lambda here stops the (String, Shooter) compiler crash!
        driverTab.addDouble("Shooter RPM", () -> m_shooter.getCurrentRPM())
        .withWidget(BuiltInWidgets.kDial)
        .withPosition(3, 0)
        .withSize(2, 2)
        .withProperties(Map.of("min", 0, "max", 6000));


        // FIX: Explicit lambda prevents the same issue here
       

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


        m_smartShootToggle = Shuffleboard
            .getTab("Driver")
            .add("Smart Shooting Enabled", constants.kEnableSmartShooting)
            .withWidget(BuiltInWidgets.kToggleSwitch)
            .getEntry();
    }



    public void setLimelightPipeline(int pipeline) {
        // UPDATED NAME HERE
        NetworkTableInstance.getDefault().getTable("limelight_turret").getEntry("pipeline").setNumber(pipeline);
    }


    public Command systemCheckCommand() {
        return Commands.sequence(
            Commands.print("Starting System Check..."),
            m_intake.runOnce(() -> m_intake.setSpeed(0.2, 0.2)).withTimeout(0.5),
            m_intake.runOnce(() -> m_intake.stop()),
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
