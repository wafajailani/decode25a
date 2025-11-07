package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.BotConstants.BLUE_GOAL_POSE;
import static org.firstinspires.ftc.teamcode.BotConstants.VELOCITY_COMPENSATION_MULTIPLIER;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.utils.MathUtil;
import org.firstinspires.ftc.teamcode.utils.ShooterSpeedLookup;
import org.firstinspires.ftc.teamcode.utils.Util;

public class ShooterSubsystem extends SubsystemBase {
    private final MotorEx shooterMotor1;
    private final MotorEx shooterMotor2;
    private double desiredSpeed = 0.0;
    private final TelemetryManager telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();

    public static double kP = 1.32;
    public static double kS = 110;
    public static double kV = 0.5;
    public static double kA = 0.66;

    private double customMultiplier = 1.0;
    private double calculated = 0.0;

    PIDFCoefficients flywheelCoefficients = new PIDFCoefficients(5.0, 0, 0.0, 0.0);
    private PIDFController flywheelPIDF = new PIDFController(flywheelCoefficients);
    private SimpleMotorFeedforward flywheelFF = new SimpleMotorFeedforward(0.10, 0.54, 0.66);
    private double CONVERT_VELOCITY_TO_MPS = 2 * Math.PI * ((double) 36 /1000) / 537.7;

    /**
     * Creates a new ShooterSubsystem.
     */
    public ShooterSubsystem(HardwareMap hardwareMap) {
        desiredSpeed = 0.0;

        shooterMotor1 = new MotorEx(hardwareMap, "shooter1");
        shooterMotor2 = new MotorEx(hardwareMap, "shooter2");

        shooterMotor1.setVelocity(0.0);
        shooterMotor2.setVelocity(0.0);

        //Setting direction, runMode for both motors
        shooterMotor1.setInverted(false);
        shooterMotor1.setRunMode(MotorEx.RunMode.VelocityControl);
        shooterMotor1.setVeloCoefficients(kP, 0.0, 0.0);
        shooterMotor1.setFeedforwardCoefficients(kS, kV);

        shooterMotor2.setInverted(true);
        shooterMotor2.setRunMode(MotorEx.RunMode.VelocityControl);
        shooterMotor2.setVeloCoefficients(kP, 0.0, 0.0);
        shooterMotor2.setFeedforwardCoefficients(kS, kV);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        shooterMotor1.setVelocity(desiredSpeed * VELOCITY_COMPENSATION_MULTIPLIER);
        shooterMotor2.setVelocity(desiredSpeed * VELOCITY_COMPENSATION_MULTIPLIER);
    }

    public void setDesiredSpeed(double speed) {
        desiredSpeed = speed;
    }

    public void setMotorDesiredVelocities(double multiplier) {
        shooterMotor1.setVelocity(desiredSpeed * multiplier);
        shooterMotor2.setVelocity(desiredSpeed * multiplier);
    }

    public void updateSpeeds(double mps, double scalar) {
        flywheelPIDF.setSetPoint(mps);
        calculated = flywheelPIDF.calculate(shooterMotor1.getCorrectedVelocity()*CONVERT_VELOCITY_TO_MPS)
                + flywheelFF.calculate(shooterMotor1.getCorrectedVelocity()*CONVERT_VELOCITY_TO_MPS);

        shooterMotor1.set(calculated);
        shooterMotor2.set(calculated);
    }



    public boolean withinFlywheelTolerance(double tolerance) {
        flywheelPIDF.setTolerance(tolerance);
        return flywheelPIDF.atSetPoint();
    }

    public double getShooter1MPS() {
        return shooterMotor1.getCorrectedVelocity() * CONVERT_VELOCITY_TO_MPS;
    }

    public Double getDesiredSpeedFromPoseAtoPoseB(Pose poseA, Pose poseB) {
        return ShooterSpeedLookup.getShooterSpeed(
                Math.abs(poseA.distanceFrom(poseB)));
    }

    public void setDesiredSpeedFromPose(Pose robotPose) {
        Double shooterSpeed = getDesiredSpeedFromPoseAtoPoseB(robotPose, new Pose(17, 128));
//        telemetryManager.addData("shooter speed", shooterSpeed);
//        telemetryManager.addData("distance", robotPose.distanceFrom(new Pose(10,137)));
//        telemetryManager.addData("isshooterspeed", isShooterAtSpeed());
//        telemetryManager.addData("shooter1speed", shooterMotor1.getVelocity());
//        telemetryManager.addData("shooter2speed", shooterMotor2.getVelocity());
//        telemetryManager.update();
        desiredSpeed=shooterSpeed;
//        shooterMotor1.setVelocity(shooterSpeed);
//        shooterMotor2.setVelocity(shooterSpeed);
    }

    public void setDesiredSpeedFromPose(Pose robotPose, Util.ALLIANCE alliance) {
        Double shooterSpeed = getDesiredSpeedFromPoseAtoPoseB(robotPose, Util.flipPoseAlliance(BLUE_GOAL_POSE, alliance));
        desiredSpeed=shooterSpeed;
    }

    public boolean isWithinTolerance(double tolerance) {
        return MathUtil.withinTolerance(getShooterSpeed(), getDesiredSpeed(), tolerance);
    }

    public double getDesiredSpeed() { return desiredSpeed; }

    public double getShooterSpeed() {
        return (Math.abs(shooterMotor1.getVelocity()) + Math.abs(shooterMotor2.getVelocity()))/2;
    }

    public double getShooterSpeedSmart() {

        //If shooter velocities averaged are less than 1200, then if shooter1 is not 0 (i.e. unplugged)
        //Then return s1 * 2; otherwise, return s2 * 2; if >1200, return shooter speeds averaged
        return (Math.abs(shooterMotor1.getVelocity()) + Math.abs(shooterMotor2.getVelocity()))/2 < 1200 ?
                Math.abs(shooterMotor1.getVelocity()) != 0 ? Math.abs(shooterMotor1.getVelocity()) : Math.abs(shooterMotor2.getVelocity()) :
                (Math.abs(shooterMotor1.getVelocity()) + Math.abs(shooterMotor2.getVelocity()))/2;
    }

    public boolean isShooterAtSpeed() {
        return MathUtil.withinTolerance(getShooterSpeedSmart(), desiredSpeed, 20.0);

    }
}