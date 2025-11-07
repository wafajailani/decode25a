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
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
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

@TeleOp(name="TuningOpMode")
//@Disabled
public class TuningOpMode extends OpMode
{
    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();
    private MotorEx tuningMotor1 = null;
    private MotorEx tuningMotor2 = null;
    private final boolean multipleTuningMotors = true;
    private MultipleTelemetry mTelemetry = null;
    private FtcDashboard dashboard = null;
    private TelemetryPacket packet = null;

    @Config
    private static class tuningConstants {
        //PID Constants
        public static double kp = 0.005;
        public static double ki = 0.001;
        public static double kd = 0;
        public static boolean includePID = true;

        //Feedforward constants
        public static double ks = 0;
        public static double kv = 0;
        public static double ka = 0;
        public static boolean includeFF = false;


        public static int reference = 0;
        public static int tolerance = 5;

        public static boolean isTuningMotor1Inverted = false;
        public static boolean isTuningMotor2Inverted = true;

        public static double tuningMotor1Velocity = 0.0;
        public static double tuningMotor2Velocity = 0.0;
        public static boolean directPower = false;

        public static String tuningMotor1Name = "leftLift";
        public static String tuningMotor2Name = "rightLift";
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
        tuningMotor1 = updateMotorConfig(tuningMotor1, tuningConstants.isTuningMotor1Inverted);
        if (multipleTuningMotors) updateMotorConfig(tuningMotor2, tuningConstants.isTuningMotor2Inverted);

        if (tuningMotor1.atTargetPosition()) {
            tuningMotor1.set(0.0);
        } else {
            tuningMotor1.set(1.0);
        }
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

    private MotorEx updateMotorConfig(MotorEx m, boolean isInverted) {
        m.setInverted(isInverted);
        m.setTargetPosition(tuningConstants.reference);
        applyTuningConstants(m);

        return m;
    }

    private void sendAllTelemetry() {
        packet.put("Status", "Run Time: " + runtime);
        packet.put("tMotor1 Pos: ", tuningMotor1.getCurrentPosition());
        packet.put("tMotor1 at Target: ", tuningMotor1.atTargetPosition());
        packet.put("tMotor1 Velocity", tuningMotor1.getVelocity());
        packet.put("tMotor1 kP", tuningMotor1.getPositionCoefficient());
        packet.put("tMotor1 targetPos", tuningConstants.reference);

        if (multipleTuningMotors) {
            packet.put("tMotor2 Pos: ", tuningMotor2.getCurrentPosition());
            packet.put("tMotor2 at Target: ", tuningMotor2.atTargetPosition());
        }

        dashboard.sendTelemetryPacket(packet);
    }

    private void initializeDashboard() {
        dashboard = FtcDashboard.getInstance();
        mTelemetry = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());
        packet = new TelemetryPacket();
    }

    private void initializeHardware(HardwareMap hMap) {
        //Initializing motor(s)
        tuningMotor1  = new MotorEx(hMap, tuningConstants.tuningMotor1Name);
        tuningMotor2 = multipleTuningMotors ? new MotorEx(hMap, tuningConstants.tuningMotor2Name) : null;

        //Setting direction, runMode for both motors
        tuningMotor1.setInverted(tuningConstants.isTuningMotor1Inverted);
        tuningMotor1.setTargetPosition(tuningConstants.reference);
        tuningMotor1.setRunMode(MotorEx.RunMode.PositionControl);
        applyTuningConstants(tuningMotor1);


        if (multipleTuningMotors) {
            tuningMotor2.setInverted(tuningConstants.isTuningMotor1Inverted);
            tuningMotor2.setTargetPosition(tuningConstants.reference);
            tuningMotor2.setRunMode(MotorEx.RunMode.PositionControl);
            applyTuningConstants(tuningMotor2);
        }
    }

    private MotorEx applyTuningConstants(MotorEx m) {
        if (tuningConstants.includePID) {
            m.setPositionCoefficient(
                    tuningConstants.kp
            );
        }
        m.setPositionTolerance(tuningConstants.tolerance);
        return m;
    }
}