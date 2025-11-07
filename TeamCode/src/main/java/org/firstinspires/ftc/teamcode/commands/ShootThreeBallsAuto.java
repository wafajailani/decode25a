package org.firstinspires.ftc.teamcode.commands;

import static org.firstinspires.ftc.teamcode.BotConstants.VELOCITY_COMPENSATION_MULTIPLIER;

import android.util.Log;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

import java.util.concurrent.TimeUnit;

public class ShootThreeBallsAuto extends CommandBase {
    ShooterSubsystem m_shooter;
    IntakeSubsystem m_intake;
    Pose m_endPose;
    ElapsedTime timeout = new ElapsedTime();
    double m_scalar = VELOCITY_COMPENSATION_MULTIPLIER;
    boolean isFinished = false;

    public ShootThreeBallsAuto(
            ShooterSubsystem shooter,
            IntakeSubsystem intake,
            Pose endPose,
            double scalar
    ) {
        m_shooter = shooter;
        m_intake = intake;
        m_endPose = endPose;
        m_scalar = scalar == 0.0 ? m_scalar : scalar;

        addRequirements(shooter, intake);
    }

    @Override
    public void initialize() {
        //Lock gate and transfer
        m_intake.unlockGate();
        m_intake.unlockTransfer();
        m_shooter.setDesiredSpeedFromPose(m_endPose);
        m_shooter.setMotorDesiredVelocities(m_scalar);
    }

    @Override
    public void execute() {
        m_intake.lockGate();
        m_intake.lockTransfer();
        //Repeat three times
        for (int i=0; i<3; i++) {

            //Wait for shooter speed to spin up (+- 300 rpm) for 500ms
            timeout.reset();
            while (
                    Math.abs((m_shooter.getDesiredSpeed() * m_scalar) - m_shooter.getShooterSpeedSmart()) > 30
                            && withinTimeoutTime(500)
            ) {
                m_shooter.setMotorDesiredVelocities(m_scalar);
                m_intake.setDutyCycle(0.8);
                Log.println(Log.INFO, this.getName(), "1. shooter error: " + (Math.abs(m_shooter.getDesiredSpeed() - m_shooter.getShooterSpeedSmart())));
            }

            //Unlock transfer then outtake for 175ms
            m_intake.setDutyCycle(-0.25);
            m_intake.unlockTransfer();
            timeout.reset();

            while (withinTimeoutTime(175)) {
                m_shooter.setMotorDesiredVelocities(m_scalar);
                m_intake.setDutyCycle(-0.25);
                Log.println(Log.INFO, this.getName(), "2. shooter velo: " + m_shooter.getShooterSpeedSmart());
            }

            //Full speed outtake
            m_intake.setDutyCycle(1.0);
            m_intake.unlockTransfer();
            timeout.reset();

            //If the shooter is within 50 of desired and less than 500ms has passed
            //If outside of 50rpm, ball is likely contacting shooter
            double startShooterSpeed = m_shooter.getShooterSpeedSmart();
            while (
                    withinTimeoutTime(1000)
                            && Math.abs(startShooterSpeed - m_shooter.getShooterSpeedSmart()) < 75
            ) {
                m_shooter.setMotorDesiredVelocities(m_scalar);
                m_intake.setDutyCycle(1.0);
                Log.println(Log.INFO, this.getName(), "3. ball sensor: " + (Math.abs(startShooterSpeed - m_shooter.getShooterSpeedSmart())));
            }
            m_intake.lockTransfer();
        }

        //Lock the transfer, unlock the gate, and eject all the balls
        m_intake.lockTransfer();
        m_intake.unlockGate();
        m_intake.setDutyCycle(-1.0);
        m_shooter.setDesiredSpeed(-200);

        isFinished = true;
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    private boolean withinTimeoutTime(double millis) {
        return timeout.time(TimeUnit.MILLISECONDS) < millis;
    }
}
