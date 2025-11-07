package org.firstinspires.ftc.teamcode.opModes;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.DeferredCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.button.Button;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.sun.tools.javac.util.List;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.commands.ShootOneBall;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.utils.Util;

@TeleOp(name = "A__Use This TeleOp")
public class MainCommandOpMode extends CommandOpMode {
    //Initialize subsystems here
    private Follower follower;
    private DriveSubsystem m_drive;
    private IntakeSubsystem m_intake;
    private TurretSubsystem m_turret;
    private ShooterSubsystem m_shooter;

    public static Pose2D blueGoalPose = new Pose2D(
            DistanceUnit.INCH, 23, 122,
            AngleUnit.DEGREES, 315);

    FieldManager panelsField = PanelsField.INSTANCE.getField();
    private Util.ALLIANCE alliance = Util.ALLIANCE.NONE;

    //Initialize gamepads here
    private GamepadEx m_driver;
    private GamepadEx m_operator;

    private final TelemetryManager telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();

    //Initialize triggers here
    @SuppressWarnings({"UnusedDeclaration", "FieldCanBeLocal"})
    private Button
            m_intakeButton,
            m_outtakeButton,
            m_autoShootButton,
            m_inlineTurretAdjustmentButton;

    @Override
    public void initialize() {
        m_driver = new GamepadEx(gamepad1);
        m_operator = new GamepadEx(gamepad2);

        follower = Constants.createFollower(hardwareMap);

        m_drive = new DriveSubsystem(follower, hardwareMap);




        m_intake = new IntakeSubsystem(hardwareMap);
        m_shooter = new ShooterSubsystem(hardwareMap);
        m_turret = new TurretSubsystem(hardwareMap, telemetryManager);

        register(m_drive, m_intake, m_shooter, m_turret);
        m_drive.setDefaultCommand(new RunCommand(() -> {}, m_drive));
        m_intake.setDefaultCommand(m_intake.defaultCommand());
        m_turret.setDefaultCommand(
                new RunCommand(() -> m_turret.setTurretAngleFromPose(m_drive.getPinpointPose(), alliance),
                        m_turret));
        m_shooter.setDefaultCommand(new RunCommand(()->m_shooter.setDesiredSpeed(0), m_shooter));

        configureButtonBindings();

        while (!opModeIsActive()) {
            if (gamepad1.right_bumper) alliance = Util.ALLIANCE.RED;
            if (gamepad1.left_bumper) alliance = Util.ALLIANCE.BLUE;
        }
    }

    /*

                          _=====_                               _=====-
                         / _____ \                             / _____ \
                       +.-'_____'-.---------------------------.-'_____'-.+
                      /   |     |  '.        S O N Y        .'  |  _  |   \
                     / ___| /|\ |___ \                     / ___| /△\ |___ \
                    / |      |      | ;  __           _   ; |           _ | ;
                    | | <---   ---> | | |__|         |_:> | ||□|       (○)| |
                    | |___   |   ___| ;SELECT       START ; |___       ___| ;
                    |\    | \|/ |    /  _     ___      _   \    | (X) |    /|
                    | \   |_____|  .','" "', |___|  ,'" "', '.  |_____|  .' |
                    |  '-.______.-' /       \ANALOG/       \  '-._____.-'   |
                    |               |  LS   |------|  RS   |                |
                    |              /\       /      \       /\               |
                    |             /  '.___.'        '.___.'  \              |
                    |            /                            \             |
                     \          /                              \           /
                      \________/                                \_________/
     */
    /**
     Controller Bindings (Driver/Gamepad1)
     <br>- LB: Open transfer
     <br>- RB: Close transfer
     <br>- LT: Outtake
     <br>- RT: Intake
     <br>- X:  Auto shoot
     <br>- ○:
     <br>- △:
     <br>- □:
     <br>- 🡡:
     <br>- 🡢
     <br>- 🡠:
     <br>- 🡣:
     <br>- LS: Translation (Robot-centric)
     <br>- RS: Rotation (Robot-centric)

     */
    public void configureButtonBindings() {
        Trigger m_rightTriggerPastTolerance = new Trigger(
                () -> m_driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5)
                .whenActive(
                        new ConditionalCommand(new InstantCommand(m_intake::lockTransfer),
                        new InstantCommand(),
                        () -> !m_driver.getButton(GamepadKeys.Button.LEFT_BUMPER)))
                .whileActiveContinuous(m_intake.setDCCommand(1.0));

        Trigger m_leftTriggerPastTolerance = new Trigger(
                () -> m_driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5)
                .whileActiveContinuous(m_intake.setDCCommand(-1.0).alongWith(new
                        RunCommand(() -> m_shooter.setDesiredSpeed(-200))));

        m_intakeButton = new GamepadButton(m_driver, GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new InstantCommand(m_intake::lockTransfer));

        m_outtakeButton = new GamepadButton(m_driver, GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(m_intake::unlockTransfer));

        m_autoShootButton = new GamepadButton(m_driver, GamepadKeys.Button.A)
                .whenPressed(new DeferredCommand(
                        () -> new ShootOneBall(m_shooter, m_intake, m_drive),
                        List.of(m_shooter, m_intake, m_drive)
                ));

        Button m_manualShoot = new GamepadButton(m_driver, GamepadKeys.Button.Y)
                .whileHeld(new RunCommand(()->m_shooter.setDesiredSpeedFromPose(m_drive.getPinpointPose(), alliance)));
    }

    @Override
    public void run() {
        super.run();
//        initializePose();

        m_drive.drive(m_driver.gamepad, m_drive.getPinpointPose().getHeading(), true, telemetryManager);

//        telemetryManager.addData("pinpoint Pose", m_drive.getPinpointPose());

        panelsField.moveCursor(m_drive.getPinpointPose().getX()-8, m_drive.getPinpointPose().getY()-8);
        panelsField.setFill("white");
        panelsField.rect(16, 16);
//        panelsField.moveCursor(10, 137);
//        panelsField.setFill("red");
//        panelsField.circle(5);
//        Util.Drawing.drawRobot(m_drive.getPinpointPose());
        telemetryManager.update();
        panelsField.update();
    }

    int lockPos = 0;
    public void initializePose() {
        Pose startPose = new Pose(144-39.0552851500531, 33.246175243393594, 0); //BR Red box
        while (lockPos < 10) {
            m_drive.getPinpoint().setPosition(Util.poseToPose2D(startPose));
            lockPos++;
        }
    }
}
