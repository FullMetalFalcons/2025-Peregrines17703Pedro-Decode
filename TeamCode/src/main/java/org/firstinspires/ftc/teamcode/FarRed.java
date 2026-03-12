package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
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

@Autonomous
public class FarRed extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    private Timer pathTimer, actionTimer, opmodeTimer;
    private Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    //private Paths paths; // Paths defined in the Paths class
    private Pose startPose = new Pose(56, 8, Math.toRadians(90));
    private Pose launchPose = new Pose(56, 16, Math.toRadians(125));
    private Pose moveToClose = new Pose(45, 35, 0);
    private Pose collectClose = new Pose(10, 35, 0);
    private Pose moveToPlayer = new Pose(10, 30, Math.toRadians(90));
    private Pose collectPlayerZone = new Pose(8
            , 10, Math.toRadians(90));
    private Pose leavePose = new Pose(56, 24, Math.toRadians(125));
    private PathChain moveClose, intakeClose, shootClose, moveMed, intakeMed, shootMed, moveFar, intakeFar, shootFar, movePlayer, collectPlayer, shootPZ,  leave;
    private Path preloaded;
    PeregrineShooter peregrineShooter = new PeregrineShooter();

    @Override
    public void init() {
        startPose = startPose.mirror();
        launchPose = launchPose.mirror();
        moveToClose = moveToClose.mirror();
        collectClose = collectClose.mirror();
        moveToPlayer = moveToPlayer.mirror();
        collectPlayerZone = collectPlayerZone.mirror();
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
        movePlayer = follower.pathBuilder()
                .addPath(new BezierLine(launchPose, moveToPlayer))
                .setLinearHeadingInterpolation(launchPose.getHeading(), moveToPlayer.getHeading()).build();
        collectPlayer = follower.pathBuilder()
                .addPath(new BezierLine(moveToPlayer, collectPlayerZone))
                .setLinearHeadingInterpolation(moveToPlayer.getHeading(), collectPlayerZone.getHeading()).build();
        shootPZ = follower.pathBuilder()
                .addPath(new BezierLine(collectPlayerZone, launchPose))
                .setLinearHeadingInterpolation(collectPlayerZone.getHeading(), launchPose.getHeading()).build();
        leave = follower.pathBuilder()
                .addPath(new BezierLine(launchPose, leavePose))
                .setLinearHeadingInterpolation(launchPose.getHeading(), leavePose.getHeading()).build();
    }




    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(preloaded);
                peregrineShooter.prepareFar();
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy() && !peregrineShooter.isBusy()) {
                    peregrineShooter.shootFarNoServo();
                    setPathState(2);
                }
                break;
            case 2:
                if (!peregrineShooter.isBusy())
                {
                    follower.followPath(moveClose);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy())
                {
                    follower.followPath(intakeClose);
                    peregrineShooter.Intake();
                    setPathState(4);
                }
                break;
            case 4:
                if (!peregrineShooter.isBusy() && !follower.isBusy())
                {
                    follower.followPath(shootClose);
                    peregrineShooter.prepareFar();
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy() && !peregrineShooter.isBusy())
                {
                    peregrineShooter.shootFarNoServo();
                    setPathState(6);
                }
                break;
            case 6:
                if (!peregrineShooter.isBusy())
                {
                    follower.followPath(movePlayer);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy())
                {
                    follower.followPath(collectPlayer, .4, true);
                    peregrineShooter.Intake();
                    setPathState(8);
                }
                break;
            case 8:
                if (!peregrineShooter.isBusy() && !follower.isBusy())
                {
                    follower.followPath(shootPZ);
                    peregrineShooter.prepareFar();
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy() && !peregrineShooter.isBusy())
                {
                    peregrineShooter.shootFarNoServo();
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