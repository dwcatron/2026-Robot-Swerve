package frc.robot.commands;

import frc.robot.subsystems.Turret;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj2.command.Command;

public class TrackTargetCommand extends Command {

    private final Turret turret;
    private final NetworkTable limelightTable;

    private final double kP = 0.02;
    private final double kD = 0.003;

    private double lastError = 0.0;

    public TrackTargetCommand(Turret turret) {
        this.turret = turret;

        limelightTable = NetworkTableInstance.getDefault().getTable("limelight_tauret");

        addRequirements(turret);
    }

    @Override
    public void initialize() {
        lastError = 0.0;
    }

    @Override
    public void execute() {
        double tx = limelightTable.getEntry("tx").getDouble(0.0);
        double tv = limelightTable.getEntry("tv").getDouble(0.0);

        if (tv < 1.0) {
            turret.stop();
            return;
        }

        double error = tx;

        double derivative = error - lastError;

        double output = (kP * error) + (kD * derivative);

        // Optional: clamp output so it doesn’t go crazy
        output = Math.max(-0.4, Math.min(0.4, output));

        // Optional: deadband to stop jitter
        if (Math.abs(error) < 1.0) {
            turret.stop();
        } else {
            turret.setSpeed(output);
        }

        lastError = error;
    }

    @Override
    public void end(boolean interrupted) {
        turret.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}