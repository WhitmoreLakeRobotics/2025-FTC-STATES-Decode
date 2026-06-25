
package org.firstinspires.ftc.teamcode.pedroPathing.DisabledPPAutons;

import static org.firstinspires.ftc.teamcode.pedroPathing.CompBotConstants.pathConstraints;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Common.Settings;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.CompBotConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.Tuning;

@Disabled
@Configurable
@Autonomous(name = "dppBlueFarCorner3Cycle", group = "ppBlue")
// @Autonomous(...) is the other common choice

public class dppBlueFarCorner3Cycle extends OpMode {
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
    public static Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private ElapsedTime pTimer;// this is for pausing at the end of a path
    //configurables for pedro
    public static double powerCreeper = 0.15;
    public  static  double powerSlow = 0.3;
    public static double powerNormal = 0.65;
    public static double powerFast = 0.8;
    //
    // poses for pedropath
    private final Pose startPose = new Pose(57, 9, Math.toRadians(90)); // Start Pose of our robot.
    public static Pose scorePose = new Pose(57, 18, Math.toRadians(114)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    public static Pose scorePoseAP = new Pose(55, 20, Math.toRadians(115)); //(24 was 20) Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    //private final Pose scorePose = new Pose(wallScoreX, wallScoreY, wallScoreH); // seeing if configurables work for this. Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    public static Pose pickup1aPose = new Pose(50, 36, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    public static Pose pickup1bPose = new Pose(0, 38, Math.toRadians(180)); // (0 was 3) (First Set) of Artifacts picked up.

    public static Pose pickup2Pose = new Pose(47, 60, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    public static Pose pickup3Pose = new Pose(24, 35, Math.toRadians(180)); // Lowest (Third Set) of Artifacts from the Spike Mark.

    public static Pose pickupCornera = new Pose(30,38,Math.toRadians(185));
    public static Pose pickupCornerb = new Pose(0,42,Math.toRadians(200));
    public static Pose pickupCornerc = new Pose(-1,4,Math.toRadians(270));
    public static Pose parkInterPosea = new Pose(15,20,Math.toRadians(175));

    public static Pose parkInLoadZonePose = new Pose(5,8,Math.toRadians(190));
    private Pose currentTargetPose = startPose;
    private Pose lastPose = startPose;
    private PathChain scorePreload;
    //private PathChain parkInZone;
    private PathChain grabPickup1, grabPickup1a, scorePickup1, parkInZonePath, pickupCornerPath1,pickupCornerPathF, scorePickupCorner; //, grabPickup2, scorePickup2, grabPickup3, scorePickup3;

    // private Path grabPickup1a;
    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
      /*  scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
        scorePreload.setHeadingConstraint(0.1);
        scorePreload.setVelocityConstraint(2.0);*/

    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();
        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1aPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1aPose.getHeading())
                .build();
        grabPickup1a = follower.pathBuilder()
                .addPath(new BezierLine(pickup1aPose, pickup1bPose))
                .setLinearHeadingInterpolation(pickup1aPose.getHeading(), pickup1bPose.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1bPose, scorePoseAP))
                .setLinearHeadingInterpolation(pickup1bPose.getHeading(), scorePoseAP.getHeading()).setHeadingConstraint(0.9)
                .build();
        pickupCornerPath1 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePoseAP, pickupCornera, pickupCornerb, pickupCornerc))
                .setLinearHeadingInterpolation(scorePoseAP.getHeading(), pickupCornerb.getHeading())
                .setLinearHeadingInterpolation(pickupCornerb.getHeading(), pickupCornerc.getHeading())
                .build();
        pickupCornerPathF = follower.pathBuilder()
                .addPath(new BezierCurve(pickupCornerb, pickupCornerc))
                .setLinearHeadingInterpolation(pickupCornerb.getHeading(), pickupCornerc.getHeading())
                .build();
        scorePickupCorner = follower.pathBuilder()
                .addPath(new BezierCurve(pickupCornerc, parkInterPosea, scorePoseAP))
                .setLinearHeadingInterpolation(pickupCornerc.getHeading(), scorePose.getHeading())
                //.setHeadingInterpolation(HeadingInterpolator.facingPoint(0,144))
                .build();
        parkInZonePath = follower.pathBuilder()
                .addPath(new BezierCurve(scorePoseAP, parkInterPosea, parkInLoadZonePose))
                //.setLinearHeadingInterpolation(scorePoseAP.getHeading(), pickupCornerb.getHeading()).setHeadingConstraint(0.9)
                .setLinearHeadingInterpolation(pickupCornerb.getHeading(), parkInLoadZonePose.getHeading()).setHeadingConstraint(0.9)
                .build();
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
    public void init() {
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

        follower =  CompBotConstants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
        follower.update();
        //  pedroPanelsTelemetry.init();

        telemetryMU = PanelsTelemetry.INSTANCE.getTelemetry();

        // disp[lay starting postition
        telemetryMU.addData("initialized postition - Update ", thisUpdate);
        // Feedback to Driver Hub for debugging
        updateTelemetry();

    }


    //Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY

    @Override
    public void init_loop() {
        // initialize robot
        robot.init_loop();

    }


    //Code to run ONCE when the driver hits PLAY

    @Override
    public void start() {
        // start robot
        runtime.reset();
        robot.start();
        opmodeTimer.resetTimer();


    }


    //Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP

    @Override
    public void loop() {

        telemetry.addData("Auton_Current_Stage ", currentStage);
        robot.autonLoop();
        follower.update();
        switch (currentStage) {
            case _unknown:
                currentStage = stage._00_preStart;
                break;

            case _00_preStart:
                currentStage = stage._20_DriveToScore;
                break;

            case _20_DriveToScore:
                if (!follower.isBusy()) {
                    follower.followPath(scorePreload, powerSlow,true);
                    lastPose = currentTargetPose;
                    currentTargetPose = scorePose;
                    // follower.update();
                    robot.launcher.cmdOutfar(); // spin up luanch motors
                    currentStage = stage._25_checkDrivetoscore;
                }
            case _25_checkDrivetoscore:
                if (!follower.isBusy()) {
                    telemetryMU.addData("Drive Complete?", follower.isBusy());
                    runtime.reset();
                    currentStage = stage._30_Shoot1; // we don't need to do the turn since heading is adjusted in path
                    runtime.reset();
                }
                break;

            case _30_Shoot1:
                if (!follower.isBusy()) {
                    // if (CommonLogic.inRange(follower.getPose().getX(), wallScoreX, xTol) &&
                    //         CommonLogic.inRange(follower.getPose().getY(), wallScoreY, yTol)) {
                    if (runtime.milliseconds() >= 1000) {
                        telemetryMU.addLine("waiting to shoot 1");

                        robot.intake.cmdFoward();
                        robot.transitionRoller.cmdSpin();
                        robot.launcherBlocker.cmdUnBlock();
                        runtime.reset();
                        currentStage = stage._40_LauncherStop;
                    }
                }
                break;

            case _40_LauncherStop:
                if (runtime.milliseconds() >= 2000) {
                    // robot.driveTrain.CmdDrive(0, 0, 0.0, 0);
                    robot.launcherBlocker.cmdBlock();
                    currentStage = stage._50_Pickup1;
                }
                break;

            case _50_Pickup1:
                if (!follower.isBusy()) {
                    follower.followPath(grabPickup1, powerNormal, true);
                    lastPose = currentTargetPose;
                    currentTargetPose = pickup1aPose;
                    currentStage = stage._55_Pickup1_Startintake;
                }
                break;

            case _55_Pickup1_Startintake:
                if (!follower.isBusy()) {
                    // follower.followPath(grabPickup1a, true);

                    robot.intake.cmdFoward();
                    currentStage = stage._60_Pickup1a;
                }
                break;

            case _60_Pickup1a:
                if (!follower.isBusy()) {
                    follower.followPath(grabPickup1a,powerSlow, true);
                    lastPose = currentTargetPose;
                    currentTargetPose = pickup1bPose;
                    currentStage = stage._70_ToScorePoseAP;
                }
                break;
            case _70_ToScorePoseAP:
                if(!follower.isBusy()){
                    follower.followPath(scorePickup1,powerNormal,true);
                    lastPose = currentTargetPose;
                    currentTargetPose = scorePose;
                    robot.launcher.cmdOutfar(); // spin up launcher motors
                    currentStage = stage._75_chkDrive_to_score_P1;
                }
                break;
            case _75_chkDrive_to_score_P1:
                if (!follower.isBusy()) {
                    telemetryMU.addData("Drive Complete?", follower.isBusy());
                    runtime.reset();
                    currentStage = stage._80_ScorePickup1; // we don't need to do the turn since heading is adjusted in path
                }
                break;

            case _80_ScorePickup1:
                if (!follower.isBusy()) {
                    //                   if (CommonLogic.inRange(follower.getPose().getX(), wallScoreX, xTol) &&
                    //                           CommonLogic.inRange(follower.getPose().getY(), wallScoreY, yTol)) {
                    if (runtime.milliseconds() >= 1000) { // let path settle
                        robot.intake.cmdFoward();
                        robot.transitionRoller.cmdSpin();
                        robot.launcherBlocker.cmdUnBlock();
                        runtime.reset();
                        currentStage = stage._90_launcherStop;
                    }
                }

                break;

            case _90_launcherStop:
                if (runtime.milliseconds() >= 1500) {
                    // robot.driveTrain.CmdDrive(0, 0, 0.0, 0);
                    robot.launcherBlocker.cmdBlock();
                    currentStage = stage._100_ToPickup_Corner1;
                }
                break;
            case _100_ToPickup_Corner1:
                if (!follower.isBusy()) {
                    follower.followPath(pickupCornerPath1, powerSlow, true);
                    lastPose = currentTargetPose;
                    currentTargetPose = pickupCornerb;
                    robot.launcherBlocker.cmdBlock();
                    runtime.reset();
                    currentStage = stage._105_PickupCorner1_pickup;
                }
                break;

            case _105_PickupCorner1_pickup:
                if (!follower.isBusy()) {
                    // follower.followPath(grabPickup1a, true);
                    if (runtime.milliseconds() >= 2000) // wait for path to settle and complete pickup
                    {
                        follower.followPath(pickupCornerPathF);
                        currentStage = stage._110_ToScore_Corner1;
                    }
                }
                break;

            case _110_ToScore_Corner1:
                if (!follower.isBusy()) {
                    follower.followPath(scorePickupCorner,powerNormal, true);
                    lastPose = currentTargetPose;
                    currentTargetPose = scorePoseAP;  //was pickup1bPose
                    robot.launcher.cmdOutfar();
                    //runtime.reset();
                    currentStage = stage._120_Score_corner1;
                }
                break;
            case _120_Score_corner1:
                if(!follower.isBusy()){
                    runtime.reset();
                    currentStage=stage._125_Score_corner_after_pause;

                }
            case _125_Score_corner_after_pause:
                if(runtime.milliseconds() >= 2000){
                    robot.launcherBlocker.cmdUnBlock();
                    robot.transitionRoller.cmdSpin();
                    robot.intake.cmdFoward();
                    runtime.reset();
                    currentStage = stage._130_LauncherStop;
                }
                break;
            case _130_LauncherStop:
                if (runtime.milliseconds() >= 2000) {
                    // robot.driveTrain.CmdDrive(0, 0, 0.0, 0);
                    robot.launcherBlocker.cmdBlock();
                    runtime.reset();
                    currentStage = stage._200_parkinLoadingZone;
                }
                break;
            case _200_parkinLoadingZone:
                if (!follower.isBusy()) {
                    if (runtime.milliseconds() >= 1000) {
                        follower.followPath(parkInZonePath, powerNormal, true);
                        lastPose = currentTargetPose;
                        currentTargetPose = parkInLoadZonePose;
                        currentStage = stage._500_End;
                    }
                }
                break;
            case _500_End:
            { //do nothing let the time run out
                //ok

            }


            break;
        }

        updateTelemetry();
    }  //  loop

    private void updateTelemetry() {
        telemetryMU.addData("Current Stage", currentStage);
        telemetryMU.addData("x", follower.getPose().getX());
        telemetryMU.addData("y", follower.getPose().getY());
        telemetryMU.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetryMU.addData("LAST Pose", lastPose);
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
        // telemetryMU.addData("current Trans", follower.getTranslationalError());
        telemetryMU.addData("Heading Constraint", follower.pathConstraints.getHeadingConstraint());

        telemetryMU.update();
          Tuning.drawCurrent();
    }

    //Code to run ONCE after the driver hits STOP

    @Override
    public void stop() {
        robot.stop();
    }

    private enum stage {
        _unknown,
        _00_preStart,
        _20_DriveToScore,
        _25_checkDrivetoscore,
        _30_Shoot1,
        _40_LauncherStop,
        _50_Pickup1,
        _55_Pickup1_Startintake,
        _60_Pickup1a,
        _70_ToScorePoseAP,
        _75_chkDrive_to_score_P1,
        _80_ScorePickup1,
        _90_launcherStop,
        _100_ToPickup_Corner1,
        _105_PickupCorner1_pickup,
        _110_ToScore_Corner1,
        _120_Score_corner1,
        _125_Score_corner_after_pause,
        _130_LauncherStop,
        _200_parkinLoadingZone,
        _500_End


    }

}


