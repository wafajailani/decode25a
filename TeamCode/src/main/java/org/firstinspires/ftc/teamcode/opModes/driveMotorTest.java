package org.firstinspires.ftc.teamcode.opModes;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp
public class driveMotorTest extends LinearOpMode {


    @Override
    public void runOpMode(){

        DcMotorEx fl = hardwareMap.get(DcMotorEx.class, "frontLeft");
        DcMotorEx fr = hardwareMap.get(DcMotorEx.class, "frontRight");
        DcMotorEx bl = hardwareMap.get(DcMotorEx.class, "backLeft");
        DcMotorEx br = hardwareMap.get(DcMotorEx.class, "backRight");
        waitForStart();
        while(opModeIsActive()){
            if(gamepad1.a){
                fl.setPower(1);
            }else if(gamepad1.b){
                fr.setPower(1);
            }else if(gamepad1.x){
                bl.setPower(1);
            } else if (gamepad1.y) {
                br.setPower(1);
            }
        }

    }
}
