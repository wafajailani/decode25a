package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.ServoEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

public class _ExampleSubsystem extends SubsystemBase {
    private MotorEx exampleMotor;
    private ServoEx exampleServo;
    /**
     * Creates a new ExampleSubsystem.
     */
    public _ExampleSubsystem(HardwareMap hardwareMap) {
        exampleMotor = new MotorEx(hardwareMap, "exampleMotor");
        exampleServo = new ServoEx(hardwareMap, "exampleServo");
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}