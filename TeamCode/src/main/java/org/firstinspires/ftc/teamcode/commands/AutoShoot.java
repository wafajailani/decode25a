package org.firstinspires.ftc.teamcode.commands;
import com.bylazar.telemetry.TelemetryManager;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelDeadlineGroup;
import com.seattlesolvers.solverslib.command.ProxyScheduleCommand;
import com.seattlesolvers.solverslib.command.RepeatCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

/**
 * An example command that uses an example subsystem.
 */
public class AutoShoot extends ParallelCommandGroup {
    private final ShooterSubsystem m_shooter;
    private final IntakeSubsystem m_intake;
    private final DriveSubsystem m_drive;
    private final TurretSubsystem m_turret;
    /**
     * Creates a new ExampleCommand.
     *
     */
    public AutoShoot(
            ShooterSubsystem shooter,
            IntakeSubsystem intake,
            DriveSubsystem drive,
            TurretSubsystem turret,
            TelemetryManager tData) {
        m_shooter = shooter;
        m_intake = intake;
        m_drive = drive;
        m_turret = turret;

        addCommands(
//              new RepeatCommand(new RunCommand(() -> m_turret.setTurretAngleFromPose(m_drive.getPinpointPose()),m_turret)),
                new InstantCommand(m_intake::lockTransfer, m_intake).withTimeout(100),
                new SequentialCommandGroup(
                        new RunCommand(() -> m_shooter.setDesiredSpeedFromPose(
                        m_drive.getPinpointPose()),
                        m_shooter)
                        .interruptOn(m_shooter::isShooterAtSpeed),
                        new InstantCommand(m_intake::unlockTransfer, m_intake)
                                .alongWith(new InstantCommand(() -> m_intake.setDutyCycle(1.0))
                                ).withTimeout(500)
                )
        );

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(
//              m_drive,
//              m_turret,
                m_shooter,
                m_intake
                );
    }
}