package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.field.PanelsField;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.drivebase.MecanumDrive;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.utils.Util;

public class DriveSubsystem extends SubsystemBase {
    private Follower follower;
    private final GoBildaPinpointDriver m_pinpoint;
    private final MotorEx fl, fr, bl, br;
    private final MecanumDrive mecanumDrive;
    private final TelemetryManager telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();


    public DriveSubsystem(Follower follower, HardwareMap hMap) {
        this.follower = follower;

        fl = new MotorEx(hMap, "frontLeft");
        fr = new MotorEx(hMap, "frontRight");
        bl = new MotorEx(hMap, "backLeft");
        br = new MotorEx(hMap, "backRight");

        mecanumDrive = new MecanumDrive(true, fl, fr, bl, br);

        m_pinpoint = hMap.get(GoBildaPinpointDriver.class, "pinpoint");
        configurePinpoint();;
    }

    public void initializeFollower(HardwareMap hMap) {
        follower = Constants.createFollower(hMap);
    }

    public GoBildaPinpointDriver getPinpoint() {
        return m_pinpoint;
    }

    @Override
    public void periodic() {
        m_pinpoint.update();
//        telemetryManager.addData("ppX", m_pinpoint.getXOffset(DistanceUnit.MM));
//        telemetryManager.addData("ppY", m_pinpoint.getYOffset(DistanceUnit.MM));
        //        updateFollowerWithPinpoint();
    }

    public void drive(Gamepad gamepad, double pinpointAngle, boolean isRobotCentric, TelemetryManager tManager) {
        mecanumDrive.driveFieldCentric(
                -gamepad.left_stick_x,
                gamepad.left_stick_y,
                -gamepad.right_stick_x,
                Math.toDegrees(pinpointAngle));
        PanelsTelemetry.INSTANCE.getTelemetry().addData("angle", pinpointAngle);
    }

//    public Pose getFollowerPose() {
//        return follower.getPose();
//    }

    public Pose getPinpointPose() {
        m_pinpoint.update();
        Pose2D pose2D = m_pinpoint.getPosition();

        return Util.pose2DToPose(pose2D);
    }

    public double getFollowerHeading() {
        return follower.getPose().getHeading();
    }

    public void configurePinpoint(){
        m_pinpoint.setOffsets(-175, 1, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1
        m_pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        m_pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);
        m_pinpoint.setYawScalar(1);
        m_pinpoint.resetPosAndIMU();
    }

    public void updateFollowerWithPinpoint() {
        Pose2D pose2D = m_pinpoint.getPosition();
        this.follower.setPose(new Pose(pose2D.getX(DistanceUnit.INCH),
                pose2D.getY(DistanceUnit.INCH),
                pose2D.getHeading(AngleUnit.RADIANS)));
    }

}
