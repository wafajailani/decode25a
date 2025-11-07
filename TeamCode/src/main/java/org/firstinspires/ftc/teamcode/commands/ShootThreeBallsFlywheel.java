//package org.firstinspires.ftc.teamcode.commands;
//
//import static org.firstinspires.ftc.teamcode.BotConstants.VELOCITY_COMPENSATION_MULTIPLIER;
//
//import android.util.Log;
//
//import com.pedropathing.geometry.Pose;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.seattlesolvers.solverslib.command.CommandBase;
//
//import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
//
//import java.util.concurrent.TimeUnit;
//
//public class ShootThreeBallsFlywheel extends CommandBase {
//    ShooterSubsystem m_shooter;
//    IntakeSubsystem m_intake;
//    double m_velocityMPS;
//    ElapsedTime timeout = new ElapsedTime();
//    double m_scalar = VELOCITY_COMPENSATION_MULTIPLIER;
//    boolean isFinished = false;
//
//    public ShootThreeBallsFlywheel(
//            ShooterSubsystem shooter,
//            IntakeSubsystem intake,
//            double mps,
//            double scalar
//    ) {
//        m_shooter = shooter;
//        m_intake = intake;
//        m_velocityMPS = mps;
//        m_scalar = scalar == 0.0 ? m_scalar : scalar;
//
//        addRequirements(intake);
//    }
//
//    @Override
//    public void initialize() {
//        //Lock gate and transfer
////        m_intake.unlockGate();
////        m_intake.unlockTransfer();
//        m_shooter.updateSpeeds(m_velocityMPS, m_scalar);
//        Log.println(Log.INFO, this.getName(), "correct");
//    }
//
//    @Override
//    public void execute() {
//        m_intake.lockGate();
//        m_intake.lockTransfer();
//
//        m_intake.setDutyCycle(-0.0);
//
//        timeout.reset();
//        while (withinTimeoutTime(1000)
//                && !m_shooter.withinFlywheelTolerance(0.1)) {
//            Log.println(Log.INFO, "Is Shooter Within Velocity:", "" + m_shooter.withinFlywheelTolerance(0.03));
//            Log.println(Log.INFO, "Shooter Velocity:", "" + m_shooter.getShooter1MPS());
//            m_intake.setDutyCycle(-0.0);
//            m_shooter.updateSpeeds(m_velocityMPS, m_scalar);
//        }
//
//        for (int i = 0; i<3; i++) {
//
//            timeout.reset();
//            while (withinTimeoutTime(250)) {
//                Log.println(Log.INFO, "Shooter Velocity:", "" + m_shooter.getShooter1MPS());
//                m_intake.setDutyCycle(-1.0);
//                m_shooter.updateSpeeds(m_velocityMPS, m_scalar);
//            }
//
//
//
//            m_intake.unlockTransfer();
//            timeout.reset();
//
//            double startSpeed =  m_shooter.getShooter1MPS();
//            while (withinTimeoutTime(500) && Math.abs(startSpeed-m_shooter.getShooter1MPS())<0.1) {
//                Log.println(Log.INFO, "Time", ""+timeout.time(TimeUnit.MILLISECONDS));
//                m_intake.setDutyCycle(1.0);
//                m_shooter.updateSpeeds(m_velocityMPS, m_scalar);
//            }
//        }
//
//        double startSpeed =  m_shooter.getShooter1MPS();
//        timeout.reset();
//        while (withinTimeoutTime(750) && Math.abs(startSpeed-m_shooter.getShooter1MPS())<0.1) {
//            Log.println(Log.INFO, "Time", ""+timeout.time(TimeUnit.MILLISECONDS));
//            m_intake.setDutyCycle(1.0);
//            m_shooter.updateSpeeds(m_velocityMPS, m_scalar);
//        }
//
//        //Lock the transfer, unlock the gate, and eject all the balls
//        m_intake.lockTransfer();
//        m_intake.unlockGate();
//        m_intake.setDutyCycle(-1.0);
//        m_shooter.setDesiredSpeed(-200);
//
//        isFinished = true;
//    }
//
//    @Override
//    public boolean isFinished() {
//        return isFinished;
//    }
//
//    private boolean withinTimeoutTime(double millis) {
//        return timeout.time(TimeUnit.MILLISECONDS) < millis;
//    }
//}

package org.firstinspires.ftc.teamcode.commands;

import static org.firstinspires.ftc.teamcode.BotConstants.VELOCITY_COMPENSATION_MULTIPLIER;

import android.util.Log;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.utils.MathUtil;

import java.util.concurrent.TimeUnit;

public class ShootThreeBallsFlywheel extends CommandBase {
    ShooterSubsystem m_shooter;
    IntakeSubsystem m_intake;
    double m_velocityMPS;
    ElapsedTime timeout = new ElapsedTime();
    double m_scalar = VELOCITY_COMPENSATION_MULTIPLIER;
    boolean isFinished = false;
    Double m_desiredVelocityMPS;
    double m_startSpeed;

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
        //Make the speeds be updated to the desired
        m_shooter.updateSpeeds(m_velocityMPS, m_scalar);
        Log.println(Log.INFO, this.getName(), "correct");

        //Lock everything and stop the intake
        m_intake.lockGate();
        m_intake.lockTransfer();
        m_intake.setDutyCycle(0.0);

        timeout.reset();
    }

    @Override
    public void execute() {

        //Repeat three times
        for (int i = 0; i<3; i++) {

            //Outtake for 250ms so balls get unstuck from the black gecko wheels
            timeout.reset();
            while (withinTimeoutTime(250)) {

                //Print out the shooter velocity and desired velocity
                Log.println(Log.INFO, this.getName(), "Shooter Velocity:" + m_shooter.getShooter1MPS());
                Log.println(Log.INFO, this.getName(), "Desired Velocity:" + (m_desiredVelocityMPS == null ? 0.0 : m_desiredVelocityMPS));

                m_intake.setDutyCycle(-1.0);
                m_shooter.updateSpeeds(m_velocityMPS, m_scalar);
            }


            //If m_desiredVelocityMPS exists, warm up for another 250ms to the desired speed
            // NOTE: shouldn't take this long, but I think our loop times are really high
            if (m_desiredVelocityMPS!=null) {
                timeout.reset();
                while (withinTimeoutTime(250) &&
                        !MathUtil.withinTolerance(m_desiredVelocityMPS-m_shooter.getShooter1MPS(), 0.025)) {

                    //Print out the shooter velocity and desired velocity
                    Log.println(
                            Log.INFO,
                            this.getName(),
                            "Shooter Velocity:" + m_shooter.getShooter1MPS());
                    Log.println(
                            Log.INFO,
                            this.getName(),
                            "Velocity Error:" + Math.abs(m_desiredVelocityMPS-m_shooter.getShooter1MPS()));

                    //Stop intake and update desired shooter speeds
                    m_intake.setDutyCycle(0.0);
                    m_shooter.updateSpeeds(m_velocityMPS, m_scalar);
                }
            }

            //Unlock the transfer
            m_intake.unlockTransfer();
            timeout.reset();

            //Get the current speed of the shooter.
            m_startSpeed =  m_shooter.getShooter1MPS();


            //If the desired velocity is unknown, set the desired velocity to this speed
            //This will be used later to determine the correct speed of subsequent shots
            //See beginning of for loop
            if (m_desiredVelocityMPS == null) m_desiredVelocityMPS = m_startSpeed;

            //Wait for 500ms or if shooter is outside of tolerance => ball in transit
            while (withinTimeoutTime(500)
                    && MathUtil.withinTolerance(m_startSpeed-m_shooter.getShooter1MPS(), 0.1)) {
//                Log.println(Log.INFO, "Time", ""+timeout.time(TimeUnit.MILLISECONDS));
                m_intake.setDutyCycle(1.0);
                m_shooter.updateSpeeds(m_velocityMPS, m_scalar);
            }
        }

        //Finally, try shooting whatever ball(s) we still have.
        //Better to discard than mess up the rest of auto
        timeout.reset();
        while (withinTimeoutTime(750)
                && MathUtil.withinTolerance(m_startSpeed-m_shooter.getShooter1MPS(), 0.1)) {
//            Log.println(Log.INFO, "Time", ""+timeout.time(TimeUnit.MILLISECONDS));
            m_intake.setDutyCycle(1.0);
            m_shooter.updateSpeeds(m_velocityMPS, m_scalar);
        }

        //Lock the transfer, unlock the gate, and eject all the balls
        //Again, consistency over potential benefits
        m_intake.lockTransfer();
        m_intake.unlockGate();
        m_shooter.setDesiredSpeed(-200);
        m_intake.setDutyCycle(-1.0);

        isFinished = true;
    }
    
    
    

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    /**
     * Helper method for determining when wait is finished
     * @param millis Time in ms
     * @return Whether time is still within millis ms
     */
    private boolean withinTimeoutTime(double millis) {
        return timeout.time(TimeUnit.MILLISECONDS) < millis;
    }
}