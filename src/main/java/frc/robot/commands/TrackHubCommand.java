package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Turret;

/**
 * Continuously rotates the turret to face the alliance hub using field-relative
 * odometry. Runs for the entire match (auton through teleop) and never finishes
 * on its own. Clamps the setpoint 2° inside the soft limits so the rotator
 * never drives into the hard stops.
 */
public class TrackHubCommand extends Command {

    private static final double SAFE_MIN_DEG = Constants.Turret.MIN_ANGLE_DEG + 2.0;
    private static final double SAFE_MAX_DEG = Constants.Turret.MAX_ANGLE_DEG - 2.0;

    private final Turret turret;
    private final CommandSwerveDrivetrain drivetrain;

    public TrackHubCommand(Turret turret, CommandSwerveDrivetrain drivetrain) {
        this.turret = turret;
        this.drivetrain = drivetrain;
        addRequirements(turret);
    }

    @Override
    public void execute() {
        Translation2d hub = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                ? Constants.Field.BLUE_HUB
                : Constants.Field.RED_HUB;

        Pose2d pose = drivetrain.getState().Pose;

        double angleRad = Math.atan2(pose.getY() - hub.getY(), pose.getX() - hub.getX());
        angleRad -= pose.getRotation().getRadians();
        double angleDeg = Math.toDegrees(angleRad);

        double clamped = MathUtil.clamp(angleDeg, SAFE_MIN_DEG, SAFE_MAX_DEG);
        turret.setSetpoint(clamped);
        turret.rotateTo();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        turret.stopRotator();
    }
}
