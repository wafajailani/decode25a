/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.seattlesolvers.solverslib.hardware.ServoEx;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.BotConstants;
/*
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp
//@Disabled
public class ShooterLUTOpMode extends OpMode
{
    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();
    private MotorEx shooter1 = null;
    private MotorEx shooter2 = null;
    private MotorEx intake = null;
    private ServoEx stop = null;



    private MultipleTelemetry mTelemetry = null;
    private FtcDashboard dashboard = null;
    private TelemetryPacket packet = null;
    
    private TelemetryManager telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();

    private double calculated;


    PIDFCoefficients flywheelCoefficients = new PIDFCoefficients(2.0, 0, 0.0, 0.0);
    private PIDFController flywheelPIDF = new PIDFController(flywheelCoefficients);
    private SimpleMotorFeedforward flywheelFF = new SimpleMotorFeedforward(0.10, 0.54, 0.66);
    private double CONVERT_VELOCITY_TO_MPS = 2 * Math.PI * ((double) 36 /1000) / 537.7;

    private double velocityMPS = 1.0;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        initializeDashboard();
        initializeHardware(hardwareMap);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {

        if (gamepad1.dpadUpWasPressed()) velocityMPS += 0.01;
        if (gamepad1.dpadDownWasPressed()) velocityMPS -= 0.01;
        if (gamepad1.dpadRightWasPressed()) velocityMPS += 0.1;
        if (gamepad1.dpadLeftWasPressed()) velocityMPS -= 0.1;
        if (gamepad1.right_bumper) stop.set(0.3);
        if (gamepad1.left_bumper) stop.set(0.0);

        if (gamepad1.a) {
            intake.set(0.5);
        } else if (gamepad1.y) {
            intake.set(1.0);
        } else if (gamepad1.b) {
            intake.set(-0.5);
        } else {
            intake.set(0.0);
        }

//        shooter1.setRunMode(Motor.RunMode.VelocityControl);
//        shooter2.setRunMode(Motor.RunMode.VelocityControl);

        calculated = flywheelPIDF.calculate(shooter1.getCorrectedVelocity()*CONVERT_VELOCITY_TO_MPS)
                + flywheelFF.calculate(shooter1.getCorrectedVelocity()*CONVERT_VELOCITY_TO_MPS);

        shooter1.set(calculated);
        shooter2.set(calculated);


        flywheelPIDF.setSetPoint(velocityMPS);

        // Show the elapsed game time and wheel power.
        sendAllTelemetry();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }


    private void updateServoPositions() {

    }

    private void sendAllTelemetry() {
        telemetryManager.addData("Status", "Run Time: " + runtime.toString());

        double converted1 = shooter1.getCorrectedVelocity()*CONVERT_VELOCITY_TO_MPS;

        telemetryManager.addData("shooter1 velocity MPS", converted1);
//        telemetryManager.addData("shooter2 velocity MPS", (shooter2.getCorrectedVelocity() * CONVERT_VELOCITY_TO_MPS));
        telemetryManager.addData("shooter1 correct velocity", shooter1.getCorrectedVelocity());
        telemetryManager.addData("shooter2 correct velocity", shooter2.getCorrectedVelocity());

        telemetryManager.addData("target velocity", velocityMPS);

        telemetryManager.addData("calculated PID", calculated);

        telemetry.addData("calculated PID", calculated);
        telemetry.addData("speed MPS", converted1);

        telemetryManager.update();
    }

    private void initializeDashboard() {
        dashboard = FtcDashboard.getInstance();
        mTelemetry = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());
        packet = new TelemetryPacket();
    }

    private void initializeHardware(HardwareMap hMap) {
        //Initializing motor(s)
        shooter1 = new MotorEx(hMap, "shooter1");
        shooter2 = new MotorEx(hMap, "shooter2");
        intake = new MotorEx(hMap, "intake");
        stop = new ServoEx(hMap, "stopper");

        //Setting direction, runMode for both motors
        shooter1.setInverted(false);
        shooter1.setRunMode(MotorEx.RunMode.VelocityControl);

        shooter2.setInverted(true);
        shooter2.setRunMode(MotorEx.RunMode.VelocityControl);

    }
}