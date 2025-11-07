package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class turretCorrectionTest extends LinearOpMode {
    @Override
    public void runOpMode(){
        Servo leftServo = hardwareMap.get(Servo.class, "leftTurret");
        Servo rightServo = hardwareMap.get(Servo.class, "rightTurret");
        IMU imu = hardwareMap.get(IMU.class, "imu");
        final int targetAngle = 270;
        double robotAngle;
        waitForStart();
        while(opModeIsActive()){
            robotAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double servoPos = targetAngle - robotAngle; //This probably needs to be adjusted for the 5 rotation servo
            leftServo.setPosition(servoPos);
            rightServo.setPosition(servoPos);

        }
    }

}
