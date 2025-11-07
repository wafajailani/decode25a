package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class VisionSubsystem extends SubsystemBase {
    private Limelight3A limelight = null;
    private LLResult result;

    /**
     * Creates a new VisionSubsystem.
     */
    public VisionSubsystem(HardwareMap hMap) {
        limelight = hMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)

        /*
         * Starts polling for data.
         */
        limelight.start();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        result = limelight.getLatestResult();
    }

    public LLResult getResult() {
        return result;
    }

    public double getTX() {
        return result.getTx();
    }

    public LLStatus getStatus() {
        return limelight.getStatus();
    }

    public Pose getBotPoseLL(double yawRadians) {
        Pose3D botpose = null;
        LLResult result = limelight.getLatestResult();
        limelight.updateRobotOrientation(Math.toDegrees(yawRadians));
        if (result != null && result.isValid()) {
                botpose = result.getBotpose_MT2();
        }
        return botpose != null ?
                new Pose(
                        botpose.getPosition().x,
                        botpose.getPosition().y,
                        yawRadians)
                : null;
    }
}