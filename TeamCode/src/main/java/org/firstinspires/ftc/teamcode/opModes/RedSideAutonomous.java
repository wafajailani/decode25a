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
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.FunctionalCommand;
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
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
@Autonomous
public class RedSideAutonomous extends CommandOpMode {
    private Follower follower;
    private DriveSubsystem m_drive;
    private IntakeSubsystem m_intake;
    private TurretSubsystem m_turret;
    private ShooterSubsystem m_shooter;

    TelemetryData telemetryData = new TelemetryData(telemetry);
    TelemetryManager telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();
    FieldManager fieldManager = PanelsField.INSTANCE.getField();

    // Poses
    private final Pose startPose = new Pose(119, 129, Math.toRadians(45.000));

    // Path chains
    private PathChain StartToShoot0,
            Shoot0ToIntake1, Intake1Through,
            EndIntakeToShoot1, Shoot1ToIntake2, Intake2Through, EndIntakeToShoot2,
            Shoot2ToIntake3, Intake3Through, EndIntakeToShoot3, TurnInPlace;
    private PathChain scorePickup1, scorePickup2, scorePickup3, park;


    public void buildPaths() {
         StartToShoot0 = follower.pathBuilder()
                .addPath(            // new Pose(144-24.750, 145.000),
                        new BezierLine( new Pose( 119, 129), new Pose(90, 102.000))  //     new BezierLine(new Pose(144-24.750, 145.000), new Pose(144-44.750, 112.000))
                )
                 .setConstantHeadingInterpolation(Math.toRadians(45))
                 .setBrakingStrength(0.6) // wf
                 .setGlobalDeceleration()
                 .build();

         Shoot0ToIntake1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(90, 102), new Pose(90, 75.000))  // (90, 85) too high
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0)) //45, 90
                 .setBrakingStrength(0.6) // wf
                .build();

         Intake1Through = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(90, 75.000), new Pose(132, 95.000)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

         EndIntakeToShoot1 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(132, 95.000), new Pose(94, 95.000)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                .build();

         Shoot1ToIntake2 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(94, 95.000), new Pose(94.000, 60.000))) //(94,55) too high up
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                .build();

         Intake2Through = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(94.000, 60.000), new Pose(165, 60.000)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

         EndIntakeToShoot2 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(165, 60.000), new Pose(94, 95)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                .build();

         Shoot2ToIntake3 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(94, 95), new Pose(94, 30)))
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                .build();

         Intake3Through = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(94, 30), new Pose(145, 30)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

         EndIntakeToShoot3 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(125, 30), new Pose(94, 95)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
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
                        .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(45))
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
        follower.setStartingPose(startPose);
        buildPaths();

        // Create the autonomous command sequence
        SequentialCommandGroup autonomousSequence = new SequentialCommandGroup(

                //Preloads
                moveAndPrepareFlywheel(StartToShoot0, 47, 0.66, 1.0), // decreasing = less left -> increasing = less right // -90 degrees points to front of field
                shootInPlaceFlywheel(StartToShoot0, 0.67, 1.0),
                //new InstantCommand(m_intake::unlockGate),

                //First line
                moveAndIntake(Shoot0ToIntake1, Intake1Through),
                new InstantCommand(m_intake::lockGate).withTimeout(150)
                        .deadlineWith( new RunCommand(()->m_intake.setDutyCycle(1.0))),
                moveAndPrepareFlywheel(EndIntakeToShoot1, 45, 0.78, 1.0),
                shootInPlaceFlywheel(EndIntakeToShoot1, 0.67, 1.0),

                //Second line
                moveAndIntake(Shoot1ToIntake2, Intake2Through),
                new InstantCommand(m_intake::lockGate).withTimeout(150)
                        .deadlineWith( new RunCommand(()->m_intake.setDutyCycle(1.0))),
                moveAndPrepareFlywheel(EndIntakeToShoot2, 45, 0.67, 1.0),
                shootInPlaceFlywheel(EndIntakeToShoot2, 0.67, 1.0),

                //Third line
                moveAndIntake(Shoot2ToIntake3, Intake3Through),
                new InstantCommand(m_intake::lockGate).withTimeout(150)
                .deadlineWith( new RunCommand(()->m_intake.setDutyCycle(1.0))),
                moveAndPrepareFlywheel(EndIntakeToShoot3, 45, 0.67, 1.0),
                shootInPlaceFlywheel(EndIntakeToShoot1, 0.63, 1.0),

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