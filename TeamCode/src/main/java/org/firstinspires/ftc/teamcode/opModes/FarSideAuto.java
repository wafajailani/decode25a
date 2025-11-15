package org.firstinspires.ftc.teamcode.opModes;

import static org.firstinspires.ftc.teamcode.BotConstants.VELOCITY_COMPENSATION_MULTIPLIER;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelDeadlineGroup;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.commands.ShootOneBall;
import org.firstinspires.ftc.teamcode.commands.ShootThreeBallsAuto;
import org.firstinspires.ftc.teamcode.commands.ShootThreeBallsFlywheel;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
@Autonomous
public class FarSideAuto extends CommandOpMode {
    private Follower follower;
    private DriveSubsystem m_drive;
    private IntakeSubsystem m_intake;
    private TurretSubsystem m_turret;
    private ShooterSubsystem m_shooter;

    TelemetryData telemetryData = new TelemetryData(telemetry);
    TelemetryManager telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();
    FieldManager fieldManager = PanelsField.INSTANCE.getField();

    private ElapsedTime pathTimer;

    // Path Chains
    private PathChain moveLeft, moveToIntake;

    public void buildPaths() {

        moveLeft = follower.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(130.000, 15.000), new Pose(80.000, 15.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(270))
                .build();

        moveToIntake = follower.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(80, 15), new Pose(130, 15))
                )
                .setConstantHeadingInterpolation(Math.toRadians(270))
                .build();

    }


    private Command moveAndPrepare(PathChain path, double angle, double scalar) {
        FollowPathCommand movePath = new FollowPathCommand(follower, path, 1.0);
        return new ParallelDeadlineGroup(
                movePath,
                new RunCommand(() -> m_shooter.setDesiredSpeedFromPose(path.endPose())),
                new RunCommand(()->m_shooter.setMotorDesiredVelocities(scalar)),
                new RunCommand(()->m_turret.setTurretAngle(angle)),
                new RunCommand(()->m_intake.setDutyCycle(0.5))
        );
    }

    private Command moveAndPrepareFlywheel(PathChain path, double angle, double mps, double scalar) {
        FollowPathCommand movePath = new FollowPathCommand(follower, path, 1.0);
        return new ParallelDeadlineGroup(
                movePath,
                new RunCommand(() -> m_shooter.setDesiredSpeedFromPose(path.endPose())),
                new RunCommand(()->m_shooter.updateSpeeds(mps, scalar)),
                new RunCommand(()->m_turret.setTurretAngle(angle)),
                new RunCommand(()->m_intake.setDutyCycle(0.5))
        );
    }

 /*   private Command IntakeForThreeSec() {
        pathTimer.reset();
        if (pathTimer.seconds() >=2) {
            return new RunCommand(() -> m_intake.setDutyCycle(0.5));
        }
        return null;
    } */


    private SequentialCommandGroup moveAndIntake(PathChain moveToBalls, PathChain intakeBalls) {
        return new SequentialCommandGroup(
                new ParallelDeadlineGroup(
                        new FollowPathCommand(follower, moveToBalls),
                        new RunCommand(() -> m_intake.setDutyCycle(-0.5))
                ),
                new ParallelDeadlineGroup(
                        new FollowPathCommand(follower, intakeBalls, 0.75),
                        new RunCommand(() -> m_intake.setDutyCycle(1.0))
                )
        );
    }

    private ParallelDeadlineGroup shootInPlace(PathChain path, double scalar) {
        return new ParallelDeadlineGroup(
                new WaitCommand((long) 1000),
                new ShootThreeBallsAuto(m_shooter,m_intake,follower.getPose(), scalar),
                new FollowPathCommand(follower, follower.pathBuilder()
                        .addPath(new BezierLine(path.endPose(), path.endPose()))
                        .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(135))
                        .build()));
    }

    private ParallelDeadlineGroup shootInPlaceFlywheel(PathChain path, double mps, double scalar) {
        return new ParallelDeadlineGroup(
//                new WaitCommand((long) 1000),
                new ShootThreeBallsFlywheel(m_shooter,m_intake, mps, scalar),
                new FollowPathCommand(follower, follower.pathBuilder()
                        .addPath(new BezierLine(path.endPose(), path.endPose()))
                        .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(135))
                        .build()));
    }

    private Command shootLoneBall(PathChain path, double mps, double scalar) {
        FollowPathCommand movePath = new FollowPathCommand(follower, path, 1.0);
        return new ParallelDeadlineGroup(
                new ShootOneBall(m_shooter, m_intake, m_drive),
                new FollowPathCommand(follower, follower.pathBuilder()
                        .addPath(new BezierLine(path.endPose(), path.endPose()))
                        .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(135))
                        .build()));
    }

    @Override
    public void initialize() {
        super.reset();

        m_intake = new IntakeSubsystem(hardwareMap);
        m_shooter = new ShooterSubsystem(hardwareMap);
        m_turret = new TurretSubsystem(hardwareMap, telemetryManager);

        m_intake.setDefaultCommand(new RunCommand(m_intake::stopMotor, m_intake));
        m_shooter.setDefaultCommand(new RunCommand(()->m_shooter.setMotorDesiredVelocities(VELOCITY_COMPENSATION_MULTIPLIER), m_shooter));

        register(m_intake, m_turret, m_shooter);
        register(m_intake, m_turret, m_shooter);

        // Initialize follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(130, 15, Math.toRadians(270)));
        buildPaths();

        // Create the autonomous command sequence
        SequentialCommandGroup autonomousSequence = new SequentialCommandGroup(
                new FollowPathCommand(follower, moveLeft),
                moveAndPrepareFlywheel(moveLeft, 185, 0.8, 1.0),
                shootInPlaceFlywheel(moveLeft, 0.90, 1.0),
                new FollowPathCommand(follower, moveToIntake),
               // IntakeForThreeSec(),
              /*  new FollowPathCommand(follower, moveLeft),
                moveAndPrepareFlywheel(moveLeft, 185, 0.7, 1.0),
                shootInPlaceFlywheel(moveLeft, 0.86, 1.0),  */
                //new RunCommand(()->m_intake.setDutyCycle(0.5)),


                //Ending stuff
                new WaitCommand(1000),
                new InstantCommand(m_intake::stopMotor),
                new InstantCommand(()->m_shooter.setDesiredSpeed(0.0))

        );

        ParallelDeadlineGroup autonomousWithTurretAdjustment = new ParallelDeadlineGroup(
                autonomousSequence, new RunCommand(() -> m_turret.update())
        );

        SequentialCommandGroup autonomousWithTurretAdjustmentAndAbort = new SequentialCommandGroup(
                autonomousWithTurretAdjustment.withTimeout((long) (29.5*1000.0))
                        .andThen(new FollowPathCommand(follower, follower.pathBuilder()
                                .addPath(new BezierLine(follower.getPose(), new Pose(90, 55)))
                                .setLinearHeadingInterpolation(follower.getHeading(), follower.getHeading())
                                .build()))

        );
        // Schedule the autonomous sequence
        schedule(autonomousWithTurretAdjustmentAndAbort);

        // Schedule the autonomous sequence
        // schedule(autonomousWithTurretAdjustment);
    }

    @Override
    public void run() {
        super.run();
        m_turret.setTurretAngle(185);
        m_turret.update();

        follower.update();

        fieldManager.moveCursor(follower.getPose().getX()-8, follower.getPose().getY()-8);
        fieldManager.setCursorHeading(45);
        fieldManager.setFill("white");
        fieldManager.rect(16, 16);
        fieldManager.moveCursor(10, 137);
        fieldManager.update();

        telemetryManager.addData("X", follower.getPose().getX());
        telemetryManager.addData("Y", follower.getPose().getY());
        telemetryManager.addData("Heading", follower.getPose().getHeading());
        telemetryManager.addData("Completion", follower.getCurrentTValue());
        telemetryManager.update();
    }
}