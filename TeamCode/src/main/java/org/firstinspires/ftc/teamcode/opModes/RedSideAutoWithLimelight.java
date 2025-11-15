//package org.firstinspires.ftc.teamcode.opModes;
//
//import com.seattlesolvers.solverslib.command.CommandOpMode;
//
//import static org.firstinspires.ftc.teamcode.BotConstants.VELOCITY_COMPENSATION_MULTIPLIER;
//
//import com.bylazar.field.FieldManager;
//import com.bylazar.field.PanelsField;
//import com.bylazar.telemetry.PanelsTelemetry;
//import com.bylazar.telemetry.TelemetryManager;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.PathChain;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.seattlesolvers.solverslib.command.Command;
//import com.seattlesolvers.solverslib.command.CommandOpMode;
//import com.seattlesolvers.solverslib.command.FunctionalCommand;
//import com.seattlesolvers.solverslib.command.InstantCommand;
//import com.seattlesolvers.solverslib.command.ParallelDeadlineGroup;
//import com.seattlesolvers.solverslib.command.RunCommand;
//import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
//import com.seattlesolvers.solverslib.command.WaitCommand;
//import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
//import com.seattlesolvers.solverslib.util.TelemetryData;
//
//import org.firstinspires.ftc.teamcode.commands.ShootOneBall;
//import org.firstinspires.ftc.teamcode.commands.ShootThreeBallsAuto;
//import org.firstinspires.ftc.teamcode.commands.ShootThreeBallsFlywheel;
//import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
//
//public class RedSideAutoWithLimelight extends CommandOpMode {
//    private Follower follower;
//    private DriveSubsystem m_drive;
//    private IntakeSubsystem m_intake;
//    private TurretSubsystem m_turret;
//    private ShooterSubsystem m_shooter;
//
//    TelemetryData telemetryData = new TelemetryData(telemetry);
//    TelemetryManager telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();
//    FieldManager fieldManager = PanelsField.INSTANCE.getField();
//
//    //Path Chains for ID 21  [PURPLE, PURPLE, GREEN]
//    public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9;
//    public void buildPaths21() {
//        Path1 = follower.pathBuilder()
//                .addPath(
//                        new BezierLine(new Pose(75.000, 115.000), new Pose(100.000, 83.000))
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
//                .build();
//
//        Path2 = follower.pathBuilder()
//                .addPath(
//                        new BezierLine(new Pose(100.000, 83.000), new Pose(120.000, 83.000))
//                )
//                .setConstantHeadingInterpolation(Math.toRadians(0))
//                .build();
//
//        Path3 = follower.pathBuilder()
//                .addPath(
//                        new BezierLine(new Pose(120.000, 83.000), new Pose(95.000, 100.000))
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
//                .build();
//
//        Path4 = follower.pathBuilder()
//                .addPath(
//                        new BezierLine(new Pose(95.000, 100.000), new Pose(95.000, 60.000))
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
//                .build();
//
//        Path5 = follower.pathBuilder()
//                .addPath(
//                        new BezierLine(new Pose(95.000, 60.000), new Pose(125.000, 60.000))
//                )
//                .setConstantHeadingInterpolation(Math.toRadians(0))
//                .build();
//
//        Path6 = follower.pathBuilder()
//                .addPath(
//                        new BezierLine(new Pose(125.000, 60.000), new Pose(82.000, 81.000))
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
//                .build();
//
//        Path7 = follower.pathBuilder()
//                .addPath(
//                        new BezierLine(new Pose(82.000, 81.000), new Pose(103.000, 35.000))
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
//                .build();
//
//        Path8 = follower.pathBuilder()
//                .addPath(
//                        new BezierLine(new Pose(103.000, 35.000), new Pose(125.000, 35.000))
//                )
//                .setConstantHeadingInterpolation(Math.toRadians(0))
//                .build();
//
//        Path9 = follower.pathBuilder()
//                .addPath(
//                        new BezierLine(new Pose(125.000, 35.000), new Pose(80.000, 10.000))
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(55))
//                .build();
//    }
//
//    // Path Chains for ID 22 [PURPLE. GREEN, PURPLE]
//
//    public PathChain GoToRow2, IntakeRow2, GoToShoot2, GoToRow1, IntakeRow1, GoToShoot1, GoToRow3, IntakeRow3, GoToShoot3;
//
//    public void buildPaths22() {
//        Path1 = follower.pathBuilder()
//                .addPath(
//                        new BezierLine(new Pose(75.000, 115.000), new Pose(102.000, 60.000))
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
//                .build();
//
//        Path2 = follower.pathBuilder()
//                .addPath(
//                        new BezierLine(new Pose(102.000, 60.000), new Pose(122.000, 60.000))
//                )
//                .setConstantHeadingInterpolation(Math.toRadians(0))
//                .build();
//
//        Path3 = follower.pathBuilder()
//                .addPath(
//                        new BezierLine(new Pose(122.000, 60.000), new Pose(102.000, 60.000))
//                )
//                .setConstantHeadingInterpolation(Math.toRadians(0))
//                .build();
//
//        Path4 = follower.pathBuilder()
//                .addPath(
//                        new BezierLine(new Pose(102.000, 60.000), new Pose(100.000, 105.000))
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
//                .build();
//
//        Path5 = follower.pathBuilder()
//                .addPath(
//                        new BezierLine(new Pose(100.000, 105.000), new Pose(100.000, 85.000))
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
//                .build();
//
//        Path7 = follower.pathBuilder()
//                .addPath(
//                        new BezierLine(new Pose(100.000, 85.000), new Pose(100.000, 85.000))
//                )
//                .setConstantHeadingInterpolation(Math.toRadians(0))
//                .build();
//
//        Path8 = follower.pathBuilder()
//                .addPath(
//                        new BezierLine(new Pose(100.000, 85.000), new Pose(115.000, 85.000))
//                )
//                .setConstantHeadingInterpolation(Math.toRadians(0))
//                .build();
//
//        Path9 = follower.pathBuilder()
//                .addPath(
//                        new BezierLine(new Pose(115.000, 85.000), new Pose(100.000, 105.000))
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
//                .build();
//
//        Path10 = follower.pathBuilder()
//                .addPath(
//                        new BezierLine(new Pose(100.000, 105.000), new Pose(100.000, 35.000))
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
//                .build();
//
//        Path11 = follower.pathBuilder()
//                .addPath(
//                        new BezierLine(new Pose(100.000, 35.000), new Pose(115.000, 35.000))
//                )
//                .setConstantHeadingInterpolation(Math.toRadians(0))
//                .build();
//
//        Path12 = follower.pathBuilder()
//                .addPath(
//                        new BezierLine(new Pose(115.000, 35.000), new Pose(80.000, 10.000))
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(55))
//                .build();
//    }
//    //Path Chains for ID 23 [GREEN, PURPLE, PURPLE]
//
//
//    private Command moveAndPrepare(PathChain path, double angle, double scalar) {
//        FollowPathCommand movePath = new FollowPathCommand(follower, path, 1.0);
//        return new ParallelDeadlineGroup(
//                movePath,
//                new RunCommand(() -> m_shooter.setDesiredSpeedFromPose(path.endPose())),
//                new RunCommand(()->m_shooter.setMotorDesiredVelocities(scalar)),
//                new RunCommand(()->m_turret.setTurretAngle(angle)),
//                new RunCommand(()->m_intake.setDutyCycle(0.5))
//        );
//    }
//
//    private Command moveAndPrepareFlywheel(PathChain path, double angle, double mps, double scalar) {
//        FollowPathCommand movePath = new FollowPathCommand(follower, path, 1.0);
//        return new ParallelDeadlineGroup(
//                movePath,
//                new RunCommand(() -> m_shooter.setDesiredSpeedFromPose(path.endPose())),
//                new RunCommand(()->m_shooter.updateSpeeds(mps, scalar)),
//                new RunCommand(()->m_turret.setTurretAngle(angle)),
//                new RunCommand(()->m_intake.setDutyCycle(0.5))
//        );
//    }
//
//    private SequentialCommandGroup moveAndIntake(PathChain moveToBalls, PathChain intakeBalls) {
//        return new SequentialCommandGroup(
//                new ParallelDeadlineGroup(
//                        new FollowPathCommand(follower, moveToBalls),
//                        new RunCommand(() -> m_intake.setDutyCycle(-0.5))
//                ),
//                new ParallelDeadlineGroup(
//                        new FollowPathCommand(follower, intakeBalls, 0.75),
//                        new RunCommand(() -> m_intake.setDutyCycle(1.0))
//                )
//        );
//    }
//
//    private ParallelDeadlineGroup shootInPlace(PathChain path, double scalar) {
//        return new ParallelDeadlineGroup(
//                new WaitCommand((long) 1000),
//                new ShootThreeBallsAuto(m_shooter,m_intake,follower.getPose(), scalar),
//                new FollowPathCommand(follower, follower.pathBuilder()
//                        .addPath(new BezierLine(path.endPose(), path.endPose()))
//                        .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(135))
//                        .build()));
//    }
//
//    private ParallelDeadlineGroup shootInPlaceFlywheel(PathChain path, double mps, double scalar) {
//        return new ParallelDeadlineGroup(
////                new WaitCommand((long) 1000),
//                new ShootThreeBallsFlywheel(m_shooter,m_intake, mps, scalar),
//                new FollowPathCommand(follower, follower.pathBuilder()
//                        .addPath(new BezierLine(path.endPose(), path.endPose()))
//                        .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(135))
//                        .build()));
//    }
//
//    private Command shootLoneBall(PathChain path, double mps, double scalar) {
//        FollowPathCommand movePath = new FollowPathCommand(follower, path, 1.0);
//        return new ParallelDeadlineGroup(
//                new ShootOneBall(m_shooter, m_intake, m_drive),
//                new FollowPathCommand(follower, follower.pathBuilder()
//                        .addPath(new BezierLine(path.endPose(), path.endPose()))
//                        .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(135))
//                        .build()));
//    }
//
//    @Override
//    public void initialize() {
//        super.reset();
//
//        m_intake = new IntakeSubsystem(hardwareMap);
//        m_shooter = new ShooterSubsystem(hardwareMap);
//        m_turret = new TurretSubsystem(hardwareMap, telemetryManager);
//
//        m_intake.setDefaultCommand(new RunCommand(m_intake::stopMotor, m_intake));
//     /*   m_turret.setDefaultCommand(
//                new RunCommand(() -> m_turret.setTurretAngleFromPose(follower.getPose()
//                        .rotate(-135, true)),
//                        m_turret)); */
//        m_shooter.setDefaultCommand(new RunCommand(()->m_shooter.setMotorDesiredVelocities(VELOCITY_COMPENSATION_MULTIPLIER), m_shooter));
//
//        register(m_intake, m_turret, m_shooter);
//        register(m_intake, m_turret, m_shooter);
//
//        // Initialize follower
//        follower = Constants.createFollower(hardwareMap);
//
//        buildPaths();
//
//        // Create the autonomous command sequence
//        SequentialCommandGroup autonomousSequence = new SequentialCommandGroup(
//
//                //Preloads
//                //new InstantCommand(m_intake::lockGate).withTimeout(150),
//
//                moveAndPrepareFlywheel(StartToShoot0, -130, 0.475, 1.0), //decreasing = less left -> increasing = less right // -90 degrees points to front of field
//                //shootLoneBall(StartToShoot0, 0.4, 1.0), // first ball
//                //shootLoneBall(StartToShoot0, 0.47, 1.0), // next two balls
//                shootInPlaceFlywheel(StartToShoot0, 0.60, 1.0),
//
//                //First line
//                new FollowPathCommand(follower, TurnInPlace),
//                moveAndIntake(Shoot0ToIntake1, Intake1Through),
//                new InstantCommand(m_intake::lockGate).withTimeout(150)
//                        .deadlineWith( new RunCommand(()->m_intake.setDutyCycle(1.0))),
//                moveAndPrepareFlywheel(EndIntakeToShoot1, -150, 0.625, 1.0),
//                shootInPlaceFlywheel(EndIntakeToShoot1, 0.625, 1.0),
//
//                //Second line
//                moveAndIntake(Shoot1ToIntake2, Intake2Through),
//                new InstantCommand(m_intake::lockGate).withTimeout(150)
//                        .deadlineWith( new RunCommand(()->m_intake.setDutyCycle(1.0))),
//                moveAndPrepareFlywheel(EndIntakeToShoot2, 32.0-90, 0.74, 1.0),
//                shootInPlaceFlywheel(EndIntakeToShoot2, 0.74, 1.0),
//
//                //Third line
//                moveAndIntake(Shoot2ToIntake3, Intake3Through),
//                new InstantCommand(m_intake::lockGate).withTimeout(150)
//                        .deadlineWith( new RunCommand(()->m_intake.setDutyCycle(1.0))),
//                moveAndPrepareFlywheel(EndIntakeToShoot3, 32.0-90, 0.74, 1.0),
//                shootInPlaceFlywheel(EndIntakeToShoot1, 0.74, 1.0),
//
//                //Ending stuff
//                new WaitCommand(1000),
//                new InstantCommand(m_intake::stopMotor),
//                new InstantCommand(()->m_shooter.setDesiredSpeed(0.0))
//
//        );
//
//        ParallelDeadlineGroup autonomousWithTurretAdjustment = new ParallelDeadlineGroup(
//                autonomousSequence, new RunCommand(() -> m_turret.update())
//        );
//
//        SequentialCommandGroup autonomousWithTurretAdjustmentAndAbort = new SequentialCommandGroup(
//                autonomousWithTurretAdjustment.withTimeout((long) (29.5*1000.0))
//                        .andThen(new FollowPathCommand(follower, follower.pathBuilder()
//                                .addPath(new BezierLine(follower.getPose(), new Pose(90, 55)))
//                                .setLinearHeadingInterpolation(follower.getHeading(), follower.getHeading())
//                                .build()))
//
//        );
//        // Schedule the autonomous sequence
//        schedule(autonomousWithTurretAdjustmentAndAbort);
//
//        // Schedule the autonomous sequence
//        // schedule(autonomousWithTurretAdjustment);
//    }
//
//    @Override
//    public void run() {
//        super.run();
//
//        follower.update();
//
//        fieldManager.moveCursor(follower.getPose().getX()-8, follower.getPose().getY()-8);
//        fieldManager.setCursorHeading(45);
//        fieldManager.setFill("white");
//        fieldManager.rect(16, 16);
//        fieldManager.moveCursor(10, 137);
//        fieldManager.update();
//
//        telemetryManager.addData("X", follower.getPose().getX());
//        telemetryManager.addData("Y", follower.getPose().getY());
//        telemetryManager.addData("Heading", follower.getPose().getHeading());
//        telemetryManager.addData("Completion", follower.getCurrentTValue());
//        telemetryManager.update();
//    }
//}