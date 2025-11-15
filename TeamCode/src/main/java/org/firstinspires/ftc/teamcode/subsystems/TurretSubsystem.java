package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.BotConstants.TURRET_AIMING_POSE;
import static org.firstinspires.ftc.teamcode.BotConstants.TURRET_ANGLE_SCALAR;

import android.util.Log;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.ServoEx;

import org.firstinspires.ftc.teamcode.utils.MathUtil;
import org.firstinspires.ftc.teamcode.utils.Util;

public class TurretSubsystem extends SubsystemBase {
    private final ServoEx leftServo;
    private final ServoEx rightServo;

    private double currentPos = 0.0;
    private double targetPos = 0.0;
    private final double maxRate; // max change per update (e.g., 0.02 per loop)


    private double targetAngle;
    private double currentAngle;
    private boolean updateTurretPeriodically = false;
    private TelemetryManager telemetryManager;
    /**
     * Creates a new ExampleSubsystem.
     */
    public TurretSubsystem(HardwareMap hardwareMap, TelemetryManager telemetryManager) {
        leftServo = new ServoEx(hardwareMap, "leftTurret");
        rightServo = new ServoEx(hardwareMap, "rightTurret");
        currentPos = getCurrPos();
        this.targetPos = getCurrPos();
        this.maxRate = 0.02;

        this.telemetryManager = telemetryManager;
    }



    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        update();
//        telemetryManager.addData("turret currAngle", currentAngle);
//        telemetryManager.addData("turret targetAngle", targetAngle);

    }

    public void setTargetPos(double targetPos) {
        this.targetPos = Math.max(0.0, Math.min(1.0, targetPos)); // clamp 0–1
        targetAngle = Util.convertTurretPosToDeg(targetPos);
    }

    public void update() {
        Log.println(Log.ASSERT, "turretUpdate", "true");
        double delta = targetPos - currentPos;
//        telemetryManager.addData("targetPos", targetPos);
        if (Math.abs(delta) > maxRate) {
            currentPos += Math.signum(delta) * maxRate;
        } else {
            currentPos = targetPos;

        }
        currentAngle = Util.convertTurretPosToDeg(currentAngle);
      //  currentAngle = Util.convertTurretPosToDeg(currentPos);

        leftServo.set(currentPos);
        rightServo.set(currentPos);
    }



    public void setTurretAngle(double degrees) {
        double degreesClamped = MathUtil.clamp(MathUtil.wrap(degrees, 0, 360), 0, 320);
//        double degreesClamped =(MathUtil.wrap(degrees, 0, 360));
        setTargetPos(Util.convertTurretDegreesToPos(degreesClamped));
        targetAngle = degreesClamped;
    }

    public void incrementAngle(double degrees) {
        targetAngle += degrees;
        setTurretAngle(targetAngle);
//        telemetry.addData("targetAngle", targetAngle);
//        telemetry.update();
    }

    public double getCurrPos(){
        return currentPos;
    }
    public double getCurrAngle(){
        return currentAngle;
    }

    public boolean isAtTargetPos() { return currentPos == targetPos; }

    public void setTurretAngleFromPose(Pose robotState) {
//        double xOffset = 72 / DistanceUnit.mmPerInch;
        Pose subtract = robotState.minus(new Pose(16, 130));
        double angleToGoal = Math.toDegrees(Math.atan2(subtract.getX(), -subtract.getY())) + 45;
//        telemetryManager.addData("angletoGoal", angleToGoal);
//        telemetryManager.addData("vector x", subtract.getX());
//        telemetryManager.addData("vector y", subtract.getY());
        double headingDeg = Math.toDegrees(robotState.getHeading());
//        double normalizedHeading =  (headingDeg > 0) ? -180 - (180 - headingDeg)
//                : (headingDeg);

//        telemetryManager.addData("current pose", robotState);
//        telemetryManager.addData("turret angle set to", Math.toDegrees(robotState.getHeading()) + angleToGoal);
//        telemetryManager.update();
        double SCALAR = 1.10;
        setTurretAngle(angleToGoal - headingDeg * SCALAR);
    }

    public void setTurretAngleFromPose(Pose robotState, Util.ALLIANCE alliance) {
        Pose subtract = robotState.minus(Util.flipPoseAlliance(TURRET_AIMING_POSE, alliance));
        double angleToGoal = Math.toDegrees(Math.atan2(subtract.getX(), -subtract.getY())) + 45;

        double headingDeg = Math.toDegrees(robotState.getHeading());


        setTurretAngle(angleToGoal - headingDeg * TURRET_ANGLE_SCALAR);
    }
}