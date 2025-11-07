package org.firstinspires.ftc.teamcode.commands;

import static org.firstinspires.ftc.teamcode.BotConstants.VELOCITY_COMPENSATION_MULTIPLIER;

import android.util.Log;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

import java.util.concurrent.TimeUnit;

public class ShootThreeBallsFlywheel extends CommandBase {
    ShooterSubsystem m_shooter;
    IntakeSubsystem m_intake;
    double m_velocityMPS;
    ElapsedTime timeout = new ElapsedTime();
    double m_scalar = VELOCITY_COMPENSATION_MULTIPLIER;
    boolean isFinished = false;

    public ShootThreeBallsFlywheel(
            ShooterSubsystem shooter,
            IntakeSubsystem intake,
            double mps,
            double scalar
    ) {
        m_shooter = shooter;
        m_intake = intake;
        m_velocityMPS = mps;
        m_scalar = scalar == 0.0 ? m_scalar : scalar;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        //Lock gate and transfer
//        m_intake.unlockGate();
//        m_intake.unlockTransfer();
        m_shooter.updateSpeeds(m_velocityMPS, m_scalar);
        Log.println(Log.INFO, this.getName(), "correct");
    }

    @Override
    public void execute() {
        m_intake.lockGate();
        m_intake.lockTransfer();

        m_intake.setDutyCycle(-0.0);

        timeout.reset();
        while (withinTimeoutTime(1000)
                && !m_shooter.withinFlywheelTolerance(0.1)) {
            Log.println(Log.INFO, "Is Shooter Within Velocity:", "" + m_shooter.withinFlywheelTolerance(0.03));
            Log.println(Log.INFO, "Shooter Velocity:", "" + m_shooter.getShooter1MPS());
            m_intake.setDutyCycle(-0.0);
            m_shooter.updateSpeeds(m_velocityMPS, m_scalar);
        }

        for (int i = 0; i<3; i++) {

            timeout.reset();
            while (withinTimeoutTime(250)) {
                Log.println(Log.INFO, "Shooter Velocity:", "" + m_shooter.getShooter1MPS());
                m_intake.setDutyCycle(-1.0);
                m_shooter.updateSpeeds(m_velocityMPS, m_scalar);
            }



            m_intake.unlockTransfer();
            timeout.reset();

            double startSpeed =  m_shooter.getShooter1MPS();
            while (withinTimeoutTime(500) && Math.abs(startSpeed-m_shooter.getShooter1MPS())<0.1) {
                Log.println(Log.INFO, "Time", ""+timeout.time(TimeUnit.MILLISECONDS));
                m_intake.setDutyCycle(1.0);
                m_shooter.updateSpeeds(m_velocityMPS, m_scalar);
            }
        }

        double startSpeed =  m_shooter.getShooter1MPS();
        timeout.reset();
        while (withinTimeoutTime(750) && Math.abs(startSpeed-m_shooter.getShooter1MPS())<0.1) {
            Log.println(Log.INFO, "Time", ""+timeout.time(TimeUnit.MILLISECONDS));
            m_intake.setDutyCycle(1.0);
            m_shooter.updateSpeeds(m_velocityMPS, m_scalar);
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
