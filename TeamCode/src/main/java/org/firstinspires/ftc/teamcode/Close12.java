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
import com.pedropathing.geometry.Pose;


import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class Close12 extends OpMode {

    public Follower follower;
    private int pathState;

    double delaySeconds = 0.0;
    final double AUTO_LENGTH_SECONDS = 30.0;
    final double AUTO_END_BUFFER_SECONDS = 1.0;


    // Define important coordinate locations for the Blue side of the field
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    //public Follower follower; // Pedro Pathing follower instance
    //private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        peregrineShooter = new PeregrineShooter();
        peregrineShooter.update();
        //pathState = autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }


    public static class Paths {
        public PathChain ShootPos1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path8;
        public PathChain Path9;

        public Paths(Follower follower) {
            ShootPos1 = follower.pathBuilder().addPath(
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
                            new BezierLine(
                                    new Pose(10.000, 60.000),

                                    new Pose(50.000, 95.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(133))

                    .build();

            Path8 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(50.000, 95.000),

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
        }
    }
    PeregrineShooter peregrineShooter;

    @Override
    public void init_loop() {

        // Modify the delay before the autonomous begins
        if (gamepad1.dpadUpWasPressed()) {
            delaySeconds += 0.5;
        }
        if (gamepad1.dpadDownWasPressed()) {
            delaySeconds -= 0.5;
        }
        telemetry.addData("Delay in seconds", delaySeconds);
        telemetry.update();

    }


    public void autonomousPathUpdate() {
        //follower.followPath(Paths, true);
        /*switch (pathState) {
            case 0:
                // Wait for the starting delay to expire
                if (delayTimer.seconds() > delaySeconds) {
                    // Begin the whole route
                    follower.followPath(launchPath1, true);
                    pathState = 1;
                }
                break;
            case 1:
                // Let the robot get to launch position

                if (!follower.isBusy()) {
                    // Begin the first launch sequence
                    //peregrineShooter.launchBallsUnprepared();
                    pathState = 2;
                }
                break;
            case 2:
                // Let the first launch sequence play out

                if (!follower.isBusy()) {
                    // drive to the first line of balls
                    follower.followPath(intakePathReady1);
                    pathState = 3;
                }
                break;
            case 3:
                // Let the robot get to the first line of balls

                if (!follower.isBusy()) {
                    // Intake the first line of balls
                    //peregrineShooter.intake.setPower(1);
                    follower.followPath(intakePath1, 0.4, true);
                    pathState = 4;
                }
                break;
            case 4:
                // Let the intake sequence play out

                if (!follower.isBusy()) {
                    follower.followPath(hitLever1);
                    pathState = 5;
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    // Stop the intake and drive back to launch position
                    //peregrineShooter.intake.setPower(0);
                    //penguinsLauncher.launcherPrepared = true;
                    //peregrineShooter.launchBallsPrepared();
                    follower.followPath(launchPath2, true);
                    pathState = 6;
                }
                break;
            case 6:
                // Let the robot get back to launch position

                if (!follower.isBusy()) {
                    // Begin the second launch sequence
                    //penguinsLauncher.fireBalls(3, true);
                    pathState = 7;
                }
                break;
            case 7:
                // Let the second launch sequence play out

                if (!follower.isBusy()) {
                    // Drive to the second line of balls
                    follower.followPath(intakePathReady2);
                    pathState = 8;
                }
                break;
            case 8:
                // Let the robot get to the second line of balls

                if (!follower.isBusy()) {
                    // Intake the second line of balls
                    //penguinsLauncher.setIntakePower(1);
                    follower.followPath(intakePath2, 0.4, true);
                    pathState = 9;
                }
                break;
            case 9:
                // Let the intake sequence play out

                if (!follower.isBusy()) {
                    // Stop the intake and drive back to launch position
                    //penguinsLauncher.setIntakePower(0);
                    //penguinsLauncher.launcherPrepared = true;
                    //penguinsLauncher.setLauncherVelocity(penguinsLauncher.velocityRpm);
                    follower.followPath(launchPath3, true);
                    pathState = 10;
                }
                break;
            case 10:
                // Let the robot get back to launch position

                if (!follower.isBusy()) {
                    // Begin the third launch sequence
                    //penguinsLauncher.fireBalls(3, true);
                    pathState = 11;
                }
                break;
            case 11:
                // Let the robot get to the third line of balls

                if (!follower.isBusy()) {
                    //penguinsLauncher.setIntakePower(0);
                    follower.followPath(intakePathReady3, true);
                    pathState = 12;
                }
                break;
            case 12:
                // Let the intake sequence play out

                if (!follower.isBusy()) {
                    //penguinsLauncher.setIntakePower(1);
                    follower.followPath(intakePath3, 0.4, true);
                    pathState = 13;
                }
                break;
            case 13:
                if (!follower.isBusy()) {
                    //penguinsLauncher.setIntakePower(0);
                    //penguinsLauncher.launcherPrepared = true;
                    //penguinsLauncher.setLauncherVelocity(penguinsLauncher.velocityRpm);
                    follower.followPath(launchPath4, true);
                    pathState = 14;
                }
                break;
            case 14:
                if (!follower.isBusy()) {
                    //penguinsLauncher.fireBalls(3, true);
                    pathState = 15;
                }
                break;

            case 15:
                // Let the third launch sequence play out

                // If the launch sequence is finished, or autonomous is about to end, move sideways for the Leave points

                    // Quit out of the state machine and move off of the Launch line
                    follower.followPath(leavePath, true);
                    pathState = -1;
                break;*/

    }
}