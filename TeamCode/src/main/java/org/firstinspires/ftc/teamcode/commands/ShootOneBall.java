package org.firstinspires.ftc.teamcode.commands;

import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

import java.util.concurrent.TimeUnit;

/**
 * An example command that uses an example subsystem.
 */
public class ShootOneBall extends CommandBase {
        ShooterSubsystem m_shooter;
        IntakeSubsystem m_intake;
        DriveSubsystem m_drive;
        ElapsedTime timeout = new ElapsedTime();

        boolean isFinished = false;

        public ShootOneBall(
                ShooterSubsystem shooter,
                IntakeSubsystem intake,
                DriveSubsystem drive
        ) {
            m_shooter = shooter;
            m_intake = intake;
            m_drive = drive;

            addRequirements(shooter, intake);
        }

        @Override
        public void execute() {
            m_intake.lockTransfer();
            m_intake.setDutyCycle(0.75);

//            PanelsTelemetry.INSTANCE.getTelemetry().addData("status", 0);

//            Log.println(Log.ASSERT, this.getName(), ("x: " + m_drive.getPinpointPose().getX() + "y: " + m_drive.getPinpointPose().getY()));

            m_shooter.setDesiredSpeedFromPose(
                    m_drive.getPinpointPose());

            m_shooter.setMotorDesiredVelocities(0.35);
//            Log.println(Log.ASSERT, this.getName(), ("" + m_shooter.getDesiredSpeed()));


            timeout.reset();
            while (!m_shooter.isWithinTolerance(100) && timeout.time(TimeUnit.MILLISECONDS) < 2000) {
//                PanelsTelemetry.INSTANCE.getTelemetry().addData("speed", m_shooter.isShooterAtSpeed() ? "1" : "0");
//                PanelsTelemetry.INSTANCE.getTelemetry().addData("motorSpeed", m_shooter.getShooterSpeed());
//                Log.println(Log.ASSERT, this.getName(), ("" + m_shooter.getDesiredSpeed()));
//                Log.println(Log.ASSERT, this.getName(), ("" + m_shooter.getShooterSpeed()));
            }

            m_intake.unlockTransfer();
            m_intake.setDutyCycle(-0.5);
            timeout.reset();
            while (timeout.time(TimeUnit.MILLISECONDS) < 100) PanelsTelemetry.INSTANCE.getTelemetry().addData("time", timeout.time(TimeUnit.MILLISECONDS));


            m_intake.setDutyCycle(1.0);

            timeout.reset();

            //If the shooter is within +-200RPM of desired and less than 500ms has passed
            while (timeout.time(TimeUnit.MILLISECONDS) < 300) {
                PanelsTelemetry.INSTANCE.getTelemetry().addData(
                        "speedError",
                        Math.abs(m_shooter.getShooterSpeed() - m_shooter.getDesiredSpeed()));
            }

            m_intake.lockTransfer();
            m_intake.setDutyCycle(0.2);

            isFinished = true;
        }

        @Override
        public boolean isFinished() {
            return isFinished;
        }
    }