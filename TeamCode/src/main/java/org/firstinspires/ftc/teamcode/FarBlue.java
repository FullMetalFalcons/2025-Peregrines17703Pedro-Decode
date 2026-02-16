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
public class FarBlue extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    private Timer pathTimer, actionTimer, opmodeTimer;
    private Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    //private Paths paths; // Paths defined in the Paths class
    private Pose startPose = new Pose(56, 8, Math.toRadians(90));
    private Pose launchPose = new Pose(56, 16, Math.toRadians(120));
    private Pose moveToClose = new Pose(45, 35, 0);
    private Pose collectClose = new Pose(10, 35, 0);
    private PathChain moveClose, intakeClose, shootClose, moveMed, intakeMed, shootMed, moveFar, intakeFar, shootFar;
    private Path preloaded;
    PeregrineShooter peregrineShooter = new PeregrineShooter();

    @Override
    public void init() {
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
                    peregrineShooter.shootFar();
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
                    follower.followPath(intakeClose, 0.5, true);
                    setPathState(-1);
                }
                break;

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