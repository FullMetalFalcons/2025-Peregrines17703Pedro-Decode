package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.Path;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class PenguinCoopRed extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    private Timer pathTimer, actionTimer, opmodeTimer;
    private ElapsedTime timer = new ElapsedTime();
    private Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    //private Paths paths; // Paths defined in the Paths class
    private Pose startPose = new Pose(14.5, 113, Math.toRadians(90));
    private Pose launchPose = new Pose(50, 95, Math.toRadians(140));
    private Pose moveToClose = new Pose(45, 83, 0);
    private Pose collectClose = new Pose(16, 83, 0);
    private Pose gateCollect = new Pose(9, 37, Math.toRadians(290));
    private Pose gateCollectControlPoint = new Pose(68, 14);
    private Pose leavePose = new Pose(45, 80, Math.toRadians(135));
    private PathChain moveClose, intakeClose, shootClose, intakeGate, shootGate, leave;
    private Path preloaded;
    PeregrineShooter peregrineShooter = new PeregrineShooter();

    @Override
    public void init() {
        startPose = startPose.mirror();
        launchPose = launchPose.mirror();
        moveToClose = moveToClose.mirror();
        collectClose = collectClose.mirror();
        gateCollect = gateCollect.mirror();
        gateCollectControlPoint = gateCollectControlPoint.mirror();
        leavePose = leavePose.mirror();

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
        peregrineShooter.init(hardwareMap);

        //paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        peregrineShooter.update();
        autonomousPathUpdate();


        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        telemetry.addData("busy?", follower.isBusy());
        telemetry.addData("busyshoot?", peregrineShooter.isBusy());
        telemetry.addData("path?", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start()
    {
        opmodeTimer.resetTimer();;
        setPathState(0);
    }

    public void buildPaths()
    {
        preloaded = new Path(new BezierLine(startPose, launchPose));
        preloaded.setLinearHeadingInterpolation(startPose.getHeading(), launchPose.getHeading());

        moveClose = follower.pathBuilder()
                .addPath(new BezierLine(launchPose, moveToClose))
                .setLinearHeadingInterpolation(launchPose.getHeading(), moveToClose.getHeading()).build();
        intakeClose = follower.pathBuilder()
                .addPath(new BezierLine(moveToClose, collectClose))
                .setLinearHeadingInterpolation(moveToClose.getHeading(), collectClose.getHeading()).build();
        shootClose = follower.pathBuilder()
                .addPath(new BezierLine(collectClose, launchPose))
                .setLinearHeadingInterpolation(collectClose.getHeading(), launchPose.getHeading()).build();
        intakeGate = follower.pathBuilder()
                .addPath(new BezierCurve(launchPose, gateCollectControlPoint, gateCollect))
                .setLinearHeadingInterpolation(launchPose.getHeading(), gateCollect.getHeading()).build();
        shootGate = follower.pathBuilder()
                .addPath(new BezierCurve(gateCollect, gateCollectControlPoint, launchPose))
                .setLinearHeadingInterpolation(gateCollect.getHeading(), launchPose.getHeading()).build();
        leave = follower.pathBuilder()
                .addPath(new BezierLine(launchPose, leavePose))
                .setLinearHeadingInterpolation(launchPose.getHeading(), leavePose.getHeading()).build();
    }


    /*public static class Paths {
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path8;
        public PathChain Path9;
        public PathChain Path10;
        public PathChain Path11;
        public PathChain Path12;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(14.500, 113.000),

                                    new Pose(50.000, 95.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(133))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(50.000, 95.000),

                                    new Pose(45.000, 84.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(133), Math.toRadians(0))

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(45.000, 84.000),

                                    new Pose(18.000, 84.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(18.000, 84.000),

                                    new Pose(50.000, 95.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(133))

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(50.000, 95.000),

                                    new Pose(45.000, 60.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(133), Math.toRadians(0))

                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(45.000, 60.000),

                                    new Pose(10.000, 60.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(10.000, 60.000),
                                    new Pose(28.000, 55.000),
                                    new Pose(50.000, 95.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(133))

                    .build();

            Path8 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(50.000, 95.000),
                                    new Pose(50.000, 79.000),
                                    new Pose(12.000, 61.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(133), Math.toRadians(-35))

                    .build();

            Path9 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(12.000, 61.000),

                                    new Pose(50.000, 95.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-35), Math.toRadians(133))

                    .build();

            Path10 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(50.000, 95.000),

                                    new Pose(40.000, 35.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(133), Math.toRadians(0))

                    .build();

            Path11 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(40.000, 35.000),

                                    new Pose(10.000, 35.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            Path12 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(10.000, 35.000),

                                    new Pose(55.000, 95.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(133))

                    .build();
        }
    }*/


    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(preloaded);
                peregrineShooter.PrepareBalls();
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy() && !peregrineShooter.isBusy()) {
                    peregrineShooter.launchBallsNoServo();
                    setPathState(2);
                }
                break;
            case 2:
                if (!peregrineShooter.isBusy()) {
                    follower.followPath(moveClose);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy())
                {
                    peregrineShooter.Intake();
                    follower.followPath(intakeClose);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy())
                {
                    follower.followPath(shootClose);
                    peregrineShooter.PrepareBalls();
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy() && !peregrineShooter.isBusy()) {
                    peregrineShooter.launchBallsNoServo();
                    setPathState(6);
                }
                break;
            case 6:
                if (!peregrineShooter.isBusy())
                {
                    follower.followPath(intakeGate);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy())
                {
                    peregrineShooter.Intake();
                    setPathState(8);
                    timer.reset();
                }
                break;
            case 8:
                if (timer.seconds() > 7.5)
                {
                    follower.followPath(shootGate);
                    peregrineShooter.PrepareBalls();
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy())
                {
                    peregrineShooter.launchBallsNoServo();
                    setPathState(10);
                }
                break;
            case 10:
                if (!peregrineShooter.isBusy())
                {
                    follower.followPath(leave);
                    setPathState(-1);
                }
        }
        telemetry.addData("stateTimer", pathTimer.getElapsedTimeSeconds());
        // Add your state machine Here
        // Access paths with paths.pathName
        // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine
    }

    public void setPathState(int pState)
    {
        pathState = pState;
        pathTimer.resetTimer();
    }
}