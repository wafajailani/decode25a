package org.firstinspires.ftc.teamcode.utils;

import androidx.annotation.Nullable;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;

import java.util.function.BooleanSupplier;

public final class MotorUtil {
    public static MotorEx initializeMotor(
            HardwareMap hMap,
            String name,
            @Nullable Motor.ZeroPowerBehavior zeroPowerBehavior,
            @Nullable BooleanSupplier isInverted,
            @Nullable Motor.RunMode runMode,
            @Nullable double[] pidConstants
    ) {
        MotorEx m = new MotorEx(hMap, name);
        m.setZeroPowerBehavior(zeroPowerBehavior != null ? zeroPowerBehavior : Motor.ZeroPowerBehavior.FLOAT);
        m.setInverted(isInverted != null && isInverted.getAsBoolean());
        m.setRunMode(runMode != null ? runMode : Motor.RunMode.RawPower);
        boolean veloNull = pidConstants != null;
        m.setVeloCoefficients(veloNull ? pidConstants[0] : 0.0, veloNull ? pidConstants[1] : 0.0, veloNull ? pidConstants[2] : 0.0);
        m.setPositionCoefficient(veloNull ? pidConstants[0]: 0.0);

        return m;
    }

    public static void setPositionPower(MotorEx m) {
        if (m.atTargetPosition()) {
            m.set(0.5);
        } else {
            m.set(1.0);
        }
    }

    public static void setPositionPower(MotorGroup m) {
        if (m.atTargetPosition()) {
            m.set(0.5);
        } else {
            m.set(1.0);
        }
    }
}
