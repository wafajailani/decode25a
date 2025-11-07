package org.firstinspires.ftc.teamcode.opModes;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="servoTester")
public class ServoTestNormal extends LinearOpMode {

    
    @Override
    public void runOpMode(){

        Servo leftServo = hardwareMap.get(Servo.class, "gate");
        Servo rightServo = hardwareMap.get(Servo.class, "rightTurret");
        leftServo.setPosition(0.35);
        waitForStart();
        leftServo.setPosition(0.6);

        while(opModeIsActive()){
            if(gamepad1.a){
                leftServo.setPosition(0.3);
                rightServo.setPosition(0.3);
            }else if(gamepad1.b){
               leftServo.setPosition(0.7);
                rightServo.setPosition(0.7);
            }else if(gamepad1.x){
                leftServo.setPosition(0.5);
                rightServo.setPosition(0.5);
            }
        }

    }
}
