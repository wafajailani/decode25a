package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.ServoEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

public class IntakeSubsystem extends SubsystemBase {
    private final MotorEx intakeMotor;
    private final ServoEx transferServo;
    private final ServoEx gateServo;
    /**
     * Creates a new IntakeSubsystem.
     */
    public IntakeSubsystem(HardwareMap hardwareMap) {
        intakeMotor = new MotorEx(hardwareMap, "intake");
        transferServo = new ServoEx(hardwareMap, "stopper");
        gateServo = new ServoEx(hardwareMap, "gate");
        intakeMotor.setInverted(false);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        PanelsTelemetry.INSTANCE.getTelemetry().addData("servo", gateServo.get());
        PanelsTelemetry.INSTANCE.getTelemetry().update();
    }

    public void setDutyCycle(double dc) {
        intakeMotor.set(dc);
    }
    public InstantCommand setDCCommand(double dc) { return new InstantCommand(() -> setDutyCycle(dc), this); }

    public void stopMotor() {
        intakeMotor.stopMotor();
    }

    public RunCommand defaultCommand() { return new RunCommand(this::stopMotor, this); }


    public void unlockTransfer() {
        transferServo.set(0.03);
    }
    public void unlockTransferAuto() {
        transferServo.set(0.1);
    }
    public void lockTransfer() { transferServo.set(0.3); }

    public void unlockGate() { gateServo.set(0.65); }
    public void lockGate() { gateServo.set(0.4); }
}