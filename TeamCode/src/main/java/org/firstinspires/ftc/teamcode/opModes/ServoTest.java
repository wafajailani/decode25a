package org.firstinspires.ftc.teamcode.opModes;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.button.Button;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

@TeleOp
public class ServoTest extends CommandOpMode {

    private TurretSubsystem m_turretSubsystem;
    //private Intake m_intakeCommand;

    private GamepadEx m_driver;
    private Button m_rightServo;
    private Button m_leftServo;

    private Button m_resetButton;
    private Button m_pos0;
    private Button m_pos1;
    private Button m_pos_default;

    private TelemetryManager telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();
    @Override
    public void initialize(){
        m_turretSubsystem = new TurretSubsystem(hardwareMap, telemetryManager);
        m_driver = new GamepadEx(gamepad1);
        m_resetButton = (new GamepadButton(m_driver,
                GamepadKeys.Button.A).whenPressed(new InstantCommand(
                () -> {m_turretSubsystem.setTurretAngle(25);}, m_turretSubsystem
        )));
        m_pos1 = (new GamepadButton(m_driver,
                GamepadKeys.Button.B).whenPressed(new InstantCommand(
                () -> {m_turretSubsystem.incrementAngle(10);}, m_turretSubsystem
        )));
        m_pos0 = (new GamepadButton(m_driver,
                GamepadKeys.Button.X).whenPressed(new InstantCommand(
                () -> {m_turretSubsystem.incrementAngle(-10);}, m_turretSubsystem
        )));
        m_pos_default = (new GamepadButton(m_driver,
                GamepadKeys.Button.Y).whenPressed(new InstantCommand(
                () -> {m_turretSubsystem.setTargetPos(0.0);}, m_turretSubsystem
        )));


        register(m_turretSubsystem);
    }
}