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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.hardware.ServoEx;
import com.seattlesolvers.solverslib.hardware.SimpleServo;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.utils.MotorUtil;
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

@TeleOp(name="PosTuningOpMode")
//@Disabled
public class PositionTuningOpMode extends OpMode
{
    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();
    private MotorEx leftLift = null;
    private MotorEx rightLift = null;
    private MotorGroup lift = null;
    private MotorEx pivot = null;
    private ServoEx elbow = null;
    private ServoEx wrist = null;
    private ServoEx claw = null;
    private MultipleTelemetry mTelemetry = null;
    private FtcDashboard dashboard = null;
    private TelemetryPacket packet = null;

    @Config
    private static class posTuningConstants {
        public static int liftRef = 0;
        public static int pivotRef = 0;
        public static int tolerance = 5;

        public static double elbowRef = 0.0;
        public static double wristRef = 0.0;
        public static double clawRef = 0.0;
    }
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

        updateMotorPositions();
        updateServoPositions();
        MotorUtil.setPositionPower(leftLift);
        MotorUtil.setPositionPower(rightLift);
        MotorUtil.setPositionPower(pivot);
        // Show the elapsed game time and wheel power.
        sendAllTelemetry();
        mTelemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    private void updateMotorPositions() {
        leftLift.setTargetPosition(posTuningConstants.liftRef);
        rightLift.setTargetPosition(posTuningConstants.liftRef);
        pivot.setTargetPosition(posTuningConstants.pivotRef);
    }

    private void updateServoPositions() {

    }

    private void sendAllTelemetry() {
        packet.put("Status", "Run Time: " + runtime.toString());

        packet.put("lift Pos: ", leftLift.getCurrentPosition());
        packet.put("lift at Target: ", leftLift.atTargetPosition());
        packet.put("lift targetPos", posTuningConstants.liftRef);

        packet.put("pivot Pos: ", pivot.getCurrentPosition());
        packet.put("pivot at Target: ", pivot.atTargetPosition());
        packet.put("pivot targetPos", posTuningConstants.pivotRef);

        dashboard.sendTelemetryPacket(packet);
    }

    private void initializeDashboard() {
        dashboard = FtcDashboard.getInstance();
        mTelemetry = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());
        packet = new TelemetryPacket();
    }

    private void initializeHardware(HardwareMap hMap) {
        //Initializing motor(s)
        leftLift = new MotorEx(hMap, "leftLift");
        rightLift = new MotorEx(hMap, "rightLift");
        pivot = new MotorEx(hMap, "pivt");

        //Setting direction, runMode for both motors
        leftLift.setInverted(false);
        leftLift.setTargetPosition(posTuningConstants.liftRef);
        leftLift.setRunMode(MotorEx.RunMode.PositionControl);
        leftLift.setPositionCoefficient(0.0075);
        leftLift.setPositionTolerance(5);



        rightLift.setInverted(true);
        rightLift.setTargetPosition(posTuningConstants.liftRef);
        rightLift.setRunMode(MotorEx.RunMode.PositionControl);
        rightLift.setPositionCoefficient(0.0075);
        rightLift.setPositionTolerance(5);

        pivot.setInverted(false);
        pivot.setTargetPosition(posTuningConstants.liftRef);
        pivot.setRunMode(MotorEx.RunMode.PositionControl);
        pivot.setPositionCoefficient(0.01);
        pivot.setPositionTolerance(5);
    }
}