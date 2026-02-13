
package org.firstinspires.ftc.teamcode.auto;
import com.pedropathing.util.Timer;
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

@Autonomous(name = "Red Near", group = "Autonomous")
@Configurable // Panels
public class RedNear extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    private final Timer pathTimer = new Timer();

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(116.60416156670745, 132.07894736842104, Math.toRadians(37)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {
        public PathChain ScorePreload;
        public PathChain GrabBalls1;
        public PathChain ScoringPos;
        public PathChain ScoreBalls1;
        public PathChain MoveOut;

        public Paths(Follower follower) {
            ScorePreload = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(116.604, 132.079),

                                    new Pose(84.761, 83.663)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-143), Math.toRadians(-136))

                    .build();

            GrabBalls1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(84.761, 83.663),

                                    new Pose(122.293, 83.414)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            ScoringPos = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(122.293, 83.414),

                                    new Pose(84.621, 83.632)
                            )
                    ).setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            ScoreBalls1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(84.621, 83.632),

                                    new Pose(84.779, 83.358)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(-132))

                    .build();

            MoveOut = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(84.779, 83.358),

                                    new Pose(95.661, 73.463)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-132), Math.toRadians(-132))

                    .build();
        }
    }


    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(paths.ScorePreload);
                setPathState(1);
                break;
            case 1:
                if(pathTimer.getElapsedTimeSeconds() >= 3) {
                    /* Score Preload */
                    follower.followPath(paths.GrabBalls1,true);
                    setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    /* Grab Balls 1 */
                    follower.followPath(paths.ScoringPos,true);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    /* Move to Scoring Position */
                    follower.followPath(paths.ScoreBalls1,true);
                    setPathState(4);
                }
                break;
            case 4:
                if(pathTimer.getElapsedTimeSeconds() >= 3) {
                    /* Score Balls 1 */
                    follower.followPath(paths.MoveOut);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    setPathState(-1);
                }
        }
    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
}
    