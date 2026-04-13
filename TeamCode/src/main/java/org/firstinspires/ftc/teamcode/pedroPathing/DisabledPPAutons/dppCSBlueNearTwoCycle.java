package org.firstinspires.ftc.teamcode.pedroPathing.DisabledPPAutons;

import static org.firstinspires.ftc.teamcode.pedroPathing.CompBotConstants.pathConstraints;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Common.Settings;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.CompBotConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.Drawing;

@Disabled
@Configurable
@Autonomous(name = "dppCSBlueNearTwoCycle", group = "PP")
// @Autonomous(...) is the other common choice

public class dppCSBlueNearTwoCycle extends OpMode {

    //RobotComp robot = new RobotComp();
    Robot robot = new Robot();
    private stage currentStage = stage._unknown;
    // declare auton power variables
    //private double AUTO_DRIVE_TURBO_SPEED = DriveTrain.DRIVETRAIN_TURBOSPEED;
    //private double AUTO_DRIVE_SLOW_SPEED = DriveTrain.DRIVETRAIN_SLOWSPEED;
    // private double AUTO_DRIVE_NORMAL_SPEED = DriveTrain.DRIVETRAIN_NORMALSPEED;
    // private double AUTO_TURN_SPEED = DriveTrain.DRIVETRAIN_TURNSPEED;

    private String RTAG = "8492-Auton";
// Set up stuff for pedro path

    private String thisUpdate = "11";
    private TelemetryManager telemetryMU;
   /* private Follower follower;*/
   public static Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private ElapsedTime pTimer;// this is for pausing at the end of a path
    //configurables for pedro
    public static int xTol = 2;  // tolorance for x axis in inches
    public static int yTol = 2; // tolorance for y axis in inches
    public static int wallScoreX = 65; //x value for scoring pose near wall
    public static int wallScoreY = 125; //y value for scoring pose near wall
    public static double wallScoreH = Math.toRadians(150);// Heading value for scoring pose near wall
   // poses for pedropath
    private final Pose startPose = new Pose(34, 135, Math.toRadians(180)); // Start Pose of our robot.
       private final Pose scorePose = new Pose(55, 135, Math.toRadians(180)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    //private final Pose scorePose = new Pose(wallScoreX, wallScoreY, wallScoreH); // seeing if configurables work for this. Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose pickup1Pose = new Pose(40, 83, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup1aPose = new Pose(24, 83, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose pickup3Pose = new Pose(24, 35, Math.toRadians(180)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    private Pose currentTargetPose = new Pose(0,0,0);

    private PathChain scorePreload;
    //private PathChain grabPickup1;//, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3;
    private PathChain grabPickup1;
    private Path grabPickup1a;
    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
       /* scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());*/
        scorePreload=follower.pathBuilder().addPath(new BezierLine(startPose,scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(),startPose.getHeading())
                .build();
        // scorePreload.setVelocityConstraint(60);
     //   scorePreload.setBrakingStrength(0.2);
     //    scorePreload.setBrakingStart(200);

    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */

        grabPickup1a = new Path(new BezierLine(pickup1Pose, pickup1aPose));
        grabPickup1a.setLinearHeadingInterpolation(pickup1Pose.getHeading(), pickup1aPose.getHeading());

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        /*grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();
        */
        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        /*scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();
        */
        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        /*grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .build();
         */
        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        /*scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, scorePose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();
        */
        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
      /*  grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup3Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .build();
        */
        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        /*scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, scorePose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .build();
    }

         */
    }
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();


    //Code to run ONCE when the driver hits INIT

    @Override
    public void init () {
        //----------------------------------------------------------------------------------------------
        // These constants manage the duration we allow for callbacks to user code to run for before
        // such code is considered to be stuck (in an infinite loop, or wherever) and consequently
        // the robot controller application is restarted. They SHOULD NOT be modified except as absolutely
        // necessary as poorly chosen values might inadvertently compromise safety.
        //----------------------------------------------------------------------------------------------
        msStuckDetectInit = Settings.msStuckDetectInit;
        msStuckDetectInitLoop = Settings.msStuckDetectInitLoop;
        msStuckDetectStart = Settings.msStuckDetectStart;
        msStuckDetectLoop = Settings.msStuckDetectLoop;
        msStuckDetectStop = Settings.msStuckDetectStop;

        robot.hardwareMap = hardwareMap;
        robot.telemetry = telemetry;
        robot.init();
        telemetry.addData("Test Auton", "Initialized");

        //Initialize Gyro
        robot.driveTrain.ResetGyro();
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        pTimer = new ElapsedTime();

        follower = CompBotConstants.createFollower(hardwareMap);
        follower.setConstraints(pathConstraints);
        buildPaths();
        follower.setStartingPose(startPose);
        follower.update();
        //  pedroPanelsTelemetry.init();
        Drawing.init();
        telemetryMU = PanelsTelemetry.INSTANCE.getTelemetry();

        // disp[lay starting postition
        telemetryMU.addData("initialized postition - Update ", thisUpdate);
        // Feedback to Driver Hub for debugging
        telemetryMU.addData("Current Stage", currentStage);
        telemetryMU.addData("x", follower.getPose().getX());
        telemetryMU.addData("y", follower.getPose().getY());
        telemetryMU.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        //  telemetryMU.addData("path", follower.getCurrentPath());
        telemetryMU.addData("y", follower.getPose().getY());
        telemetryMU.addData("heading", follower.getPose().getHeading());
        telemetryMU.addData("pose", follower.poseTracker);
        telemetryMU.addData("pose history", scorePose);
        telemetryMU.addData("breakingStrength", pathConstraints.getBrakingStrength());


        telemetryMU.update();
        Drawing.drawDebug(follower);

    }


    //Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY

    @Override
    public void init_loop () {
        // initialize robot
        robot.init_loop();

    }


    //Code to run ONCE when the driver hits PLAY

    @Override
    public void start () {
        // start robot
        runtime.reset();
        robot.start();
        opmodeTimer.resetTimer();


    }


    //Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP

    @Override
    public void loop () {

        telemetry.addData("Auton_Current_Stage ", currentStage);
        robot.autonLoop();
        follower.update();
        switch (currentStage) {
            case _unknown:
                currentStage = stage._00_preStart;
                break;
            case _00_preStart:
                currentStage = stage._20_DriveBack;
                break;


            case _20_DriveBack:
                if (!follower.isBusy()) {
                    follower.followPath(scorePreload, 0.4,true);
 //                   robot.launcher.cmdOuttouch();
                    currentStage = stage._30_Shoot1; // we don't need to do the turn since heading is adjusted in path
                }
                break;
            case _30_Shoot1:
                if (!follower.isBusy()){
                    // if (CommonLogic.inRange(follower.getPose().getX(), wallScoreX, xTol) &&
                    //         CommonLogic.inRange(follower.getPose().getY(), wallScoreY, yTol)) {
                    //robot.intake.cmdFoward();
                    //robot.transitionRoller.cmdSpin();
                    //robot.launcherBlocker.cmdUnBlock();
                    runtime.reset();
                    currentStage = stage._40_LauncherStop;
                }
                break;
            case _40_LauncherStop:
                if (runtime.time() >= 5.0) {
                    // robot.driveTrain.CmdDrive(0, 0, 0.0, 0);
                    //robot.launcherBlocker.cmdBlock();
                    currentStage = stage._50_Pickup1;
                }
                break;
            case _50_Pickup1:
                if (!follower.isBusy()){
                    follower.followPath(grabPickup1a, true);
                    currentStage=stage._500_End;
                }
                break;

            case _500_End:
                //if (robot.driveTrain.getCmdComplete()) {
                //    robot.stop();
                //}
                //Do nothing
                break;
        }

        telemetryMU.addData("Current Stage", currentStage);
        telemetryMU.addData("follower is busy?", follower.isBusy());
        telemetryMU.addData("x", follower.getPose().getX());
        telemetryMU.addData("y", follower.getPose().getY());
        telemetryMU.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetryMU.addData("ScorePose ", scorePose);
        telemetryMU.addData("breakingStrength", pathConstraints.getBrakingStrength());
        telemetryMU.addData("breakstart ", pathConstraints.getBrakingStart());
        telemetryMU.addData("drivepid P", follower.constants.coefficientsDrivePIDF.P );


        telemetryMU.update();
        Drawing.drawDebug(follower);
    }  //  loop


    //Code to run ONCE after the driver hits STOP

    @Override
    public void stop () {
        robot.stop();
    }
    private void updateTelemetry() {
        telemetryMU.addData("Current Stage", currentStage);
        telemetryMU.addData("x", follower.getPose().getX());
        telemetryMU.addData("y", follower.getPose().getY());
        telemetryMU.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetryMU.addData("Current Target Pose", currentTargetPose);
        telemetryMU.addData("breakingStrength", pathConstraints.getBrakingStrength());
        telemetryMU.addData("breakstart ", pathConstraints.getBrakingStart());
        telemetryMU.addData("drivepid P", follower.constants.coefficientsDrivePIDF.P );
        telemetryMU.addData("drivepid D", follower.constants.coefficientsDrivePIDF.D );
        telemetryMU.addData("drivepid F", follower.constants.coefficientsDrivePIDF.F );
        telemetryMU.addData("CONSTRAINTS", "");
        telemetryMU.addData("Tvalue (% complete)", follower.pathConstraints.getTValueConstraint());
        telemetryMU.addData("Current tValue", follower.getCurrentTValue());
        telemetryMU.addData("Velocity Constraint", follower.pathConstraints.getVelocityConstraint());
        telemetryMU.addData("Current Velocity", follower.getVelocity());
        telemetryMU.addData("Trans constraint", follower.pathConstraints.getTranslationalConstraint());
        telemetryMU.addData("current Trans Error", follower.getTranslationalError());
        telemetryMU.addData("Heading Constraint", follower.pathConstraints.getHeadingConstraint());
        telemetryMU.addData("current Heading error", follower.getHeadingError());
        telemetryMU.addData("Timeout Constraint", follower.pathConstraints.getTimeoutConstraint());


        telemetryMU.update();
        Drawing.drawDebug(follower);
    }
    private enum stage {
        _unknown,
        _00_preStart,
        _10_turn,
        _20_DriveBack,
        _25_Turn,
        _30_Shoot1,
        _40_LauncherStop,
        _50_Pickup1,
        _500_End


    }

}


