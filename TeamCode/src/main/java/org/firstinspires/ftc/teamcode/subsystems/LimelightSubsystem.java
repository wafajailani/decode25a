package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;

import com.seattlesolvers.solverslib.command.SubsystemBase;

import java.util.List;

public class LimelightSubsystem extends SubsystemBase {
    private final Limelight3A limelight;

    public LimelightSubsystem() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // how often we ask ll for data (100x per second)
        limelight.start();
        limelight.pipelineSwitch(0);
    }

    @Override
    public void periodic() { }

    public void getInfo() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            double tx = result.getTx(); // How far left or right the target is (degrees)
            double ty = result.getTy(); // How far up or down the target is (degrees)
            double ta = result.getTa(); // How big the target looks (0%-100% of the image)

            PanelsTelemetry.INSTANCE.getTelemetry().addData("Target X", tx);
            PanelsTelemetry.INSTANCE.getTelemetry().addData("Target Y", ty);
            PanelsTelemetry.INSTANCE.getTelemetry().addData("Target Area", ta);
        } else {
            PanelsTelemetry.INSTANCE.getTelemetry().addData("No targets", 0);
        }
    }

    public void getFiducialID() {
        LLResult result = limelight.getLatestResult();
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            int id = fiducial.getFiducialId(); // The ID number of the fiducial
//            double x = detection.getTargetXDegrees(); // Where it is (left-right)
//            double y = detection.getTargetYDegrees(); // Where it is (up-down)
//            double StrafeDistance_3D = fiducial.getRobotPoseTargetSpace().getY();
            PanelsTelemetry.INSTANCE.getTelemetry().addData("Fiducial: ", id);
        }
    }




/**
 public void setDutyCycle(double dc) {
 intakeMotor.set(dc);
 }
 public InstantCommand setDCCommand(double dc) { return new InstantCommand(() -> setDutyCycle(dc), this); }
 **/


}
