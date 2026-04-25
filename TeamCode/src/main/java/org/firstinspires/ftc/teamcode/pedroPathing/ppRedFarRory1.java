package org.firstinspires.ftc.teamcode.pedroPathing;


import static org.firstinspires.ftc.teamcode.pedroPathing.CompBotConstants.pathConstraints;


import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
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



@Configurable
@Autonomous(name = "ppRedFar7Cycle", group = "PP")
// @Autonomous(...) is the other common choice


public class ppRedFarRory1 extends OpMode {


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
    //Private Follower follower;
    public static Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private ElapsedTime pTimer;// this is for pausing at the end of a path
    //configurables for pedro
    public static double powerCreeper = 0.15;
    public  static  double powerSlow = 0.3;
    public static double powerMedium = 0.5;
    public static double powerNormal = 0.65;
    public static double powerFast = 0.8;
    // poses for pedropath
    // poses for pedropath
    public static Pose currentPose = new Pose(follower.getPose().getX(),follower.getPose().getY(),follower.getPose().getHeading());

    public static Pose startPose = new Pose(57, 9, Math.toRadians(57)); // Start Pose of our robot.
    public static Pose scorePose = new Pose(57, 15, Math.toRadians(112)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    //private final Pose scorePose = new Pose(wallScoreX, wallScoreY, wallScoreH); // seeing if configurables work for this. Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    public static Pose scorePoseAP =new Pose(52,18,Math.toRadians(10));
    public static Pose pickup1aPose = new Pose(120, 8, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    public static Pose pickup1bPose = new Pose(135, 8, Math.toRadians(190)); // (First Set) of Artifacts picked up.
    public static Pose pickup1bPoseC = new Pose(23, 27, Math.toRadians(200));
    public static Pose pickup1cPoseC = new Pose(4, 13.5, Math.toRadians(190));
    public static Pose Spike1a = new Pose(103,36, Math.toRadians(180));
    public static Pose Spike1b = new Pose(130,36,Math.toRadians(180));

    public static Pose Pickuptunnel1 = new Pose(130,37, Math.toRadians(45));
    public static Pose PickupCorner = new Pose(133, 30, Math.toRadians(5));

    public static Pose pickup2aPose = new Pose(10, 37, Math.toRadians(190)); // 10 was 8 Middle (Second Set) of Artifacts from the Spike Mark.
    public static Pose pickup2aPoseC = new Pose(71, 39, Math.toRadians(190)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    //public static Pose pickup3aPose = new Pose(47, 60, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    //public static Pose pickup3bPose = new Pose(15, 35, Math.toRadians(180)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    public static Pose endPose = new Pose(11,15,Math.toRadians(180));
    public static Pose endPose2 = new Pose(11,15,Math.toRadians(180));

    public static Pose Park = new Pose(15,10,Math.toRadians(90));


    private Pose currentTargetPose = startPose;
    private Pose lastPose = startPose;

    private PathChain Spike3;
    private PathChain Pickup1, Tunnelpickupa,PickupCorner1,ParkPath,ScorePreload;


    // private Path grabPickup1a;
    public void buildPaths() {


        class Paths {



            public Paths(Follower follower) {

                ParkPath = follower.pathBuilder()
                        .addPath(new BezierLine(currentPose, Park))
                        .setLinearHeadingInterpolation(currentPose.getHeading(), Park.getHeading())
                        .build();

                Spike3 = follower.pathBuilder()
                        .addPath(new BezierLine(scorePose,Spike1a))
                        .setLinearHeadingInterpolation(scorePose.getHeading(), Spike1a.getHeading())

                        .addPath(new BezierLine(Spike1a, Spike1b))
                        .setLinearHeadingInterpolation(Spike1a.getHeading(),Spike1b.getHeading())

                        .addPath(new BezierLine(Spike1b,scorePose))
                        .setLinearHeadingInterpolation(Spike1b.getHeading(),scorePose.getHeading())
                        .build();


                Pickup1 = follower.pathBuilder()
                        .addPath(new BezierLine(scorePose,pickup1aPose))
                        .setLinearHeadingInterpolation(scorePose.getHeading(),pickup1aPose.getHeading())

                        .addPath(new BezierLine(pickup1aPose, pickup1bPose))
                        .setLinearHeadingInterpolation(pickup1aPose.getHeading(), pickup1bPose.getHeading())

                        .addPath(new BezierLine(pickup1bPose, scorePose))
                        .setLinearHeadingInterpolation(pickup1bPose.getHeading(),scorePose.getHeading())
                        .build();


                Tunnelpickupa = follower.pathBuilder()
                        .addPath(new BezierLine(scorePose, Pickuptunnel1))
                        .setLinearHeadingInterpolation(scorePose.getHeading(), Pickuptunnel1.getHeading())

                        .addPath(new BezierLine(Pickuptunnel1,scorePose))
                        .setLinearHeadingInterpolation(Pickuptunnel1.getHeading(),scorePose.getHeading())
                        .build();

                ScorePreload = follower.pathBuilder()
                        .addPath(new BezierLine(lastPose,scorePose))
                        .setLinearHeadingInterpolation(lastPose.getHeading(),scorePose.getHeading())
                        .build();

                PathChain TunnelCorner = follower.pathBuilder()
                        .addPath(new BezierLine(scorePose, PickupCorner))
                        .setLinearHeadingInterpolation(scorePose.getHeading(), PickupCorner.getHeading())

                        .addPath(new BezierLine(PickupCorner, scorePose))
                        .setLinearHeadingInterpolation(PickupCorner.getHeading(), scorePose.getHeading())
                        .build();






              /*  Score2 = follower.pathBuilder().addPath(


                                new BezierLine(
                                        new Pose(130.382, 36.550),


                                        new Pose(76.794, 11.084)
                                )
                        ).setTangentHeadingInterpolation()
                        .setReversed()
                        .build();

               */


              /*  Tunnel3 = follower.pathBuilder().addPath(
                                new BezierCurve(
                                        new Pose(76.794, 11.084),
                                        new Pose(96.515, 29.221),
                                        new Pose(141.153, 23.527)
                                )
                        ).setTangentHeadingInterpolation()


                        .build();


                Score4 = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(141.153, 23.527),


                                        new Pose(76.366, 11.573)
                                )
                        ).setConstantHeadingInterpolation(Math.toRadians(57))


                        .build();


                Corner5 = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(76.366, 11.573),


                                        new Pose(140.137, 7.176)
                                )
                        ).setTangentHeadingInterpolation()


                        .build();


                Score6 = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(140.137, 7.176),


                                        new Pose(77.588, 9.267)
                                )
                        ).setConstantHeadingInterpolation(Math.toRadians(57))


                        .build();


                Tunnel7 = follower.pathBuilder().addPath(
                                new BezierCurve(
                                        new Pose(77.588, 9.267),
                                        new Pose(96.679, 28.924),
                                        new Pose(141.588, 23.496)
                                )
                        ).setTangentHeadingInterpolation()


                        .build();


                Score8 = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(141.588, 23.496),


                                        new Pose(78.122, 9.947)
                                )
                        ).setConstantHeadingInterpolation(Math.toRadians(57))


                        .build();


                Corner9 = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(78.122, 9.947),


                                        new Pose(140.847, 21.977)
                                )
                        ).setConstantHeadingInterpolation(Math.toRadians(1))


                        .build();


                Park10 = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(140.847, 21.977),


                                        new Pose(106.679, 9.076)
                                )
                        ).setTangentHeadingInterpolation()


                        .build();

               */
            }
        }






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
        PanelsConfigurables.INSTANCE.refreshClass(this);
        follower.setStartingPose(startPose);
        follower.update();
        //  pedroPanelsTelemetry.init();
        Drawing.init();
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
                currentStage = stage._10_Prelaunch;
                break;


            case _10_Prelaunch:
                if (!follower.isBusy()) {
                    follower.followPath(ScorePreload, powerNormal, true);
                    lastPose = startPose;
                    currentTargetPose = scorePose;
                    // follower.update();
                    robot.launcher.cmdOutfar();
                    currentStage = stage._16_Spike;
                }
                break;
            case _15_Launch:
                if (runtime.milliseconds() >= 1500) {
                    // robot.driveTrain.CmdDrive(0, 0, 0.0, 0);
                    dolaunch_process();
                    // currentStage = stage._50_Pickup1;
                    currentStage = stage._16_Spike;
                }

                break;

            case _16_Spike:
                if (!follower.isBusy()) {
                    endlaunch_process();
                    follower.followPath(Spike3);
                    currentStage = stage._25_DriveBack; // we don't need to do the turn since heading is adjusted in path
                    runtime.reset();
                }
                break;


            case _25_DriveBack:
                if (!follower.isBusy()) {
                    if (runtime.milliseconds() >= 500) {
                        telemetryMU.addLine("wqiting to shoot 1");
                        // if (CommonLogic.inRange(follower.getPose().getX(), wallScoreX, xTol) &&
                        //         CommonLogic.inRange(follower.getPose().getY(), wallScoreY, yTol)) {
                        robot.intake.cmdFoward();
                        robot.transitionRoller.cmdSpin();
                        runtime.reset();
                        currentStage = stage._27_Prelaunch;
                    }}
                break;
            case _27_Prelaunch:
                if (!follower.isBusy()) {
                    // follower.followPath(scorePreload, powerNormal, true);
                    lastPose = startPose;
                    currentTargetPose = scorePose;
                    // follower.update();
                    robot.launcher.cmdOutfar();
                    currentStage = stage._30_Shoot;
                }


break;
            case _30_Shoot:
                if (runtime.milliseconds() >= 1500) {
                    // robot.driveTrain.CmdDrive(0, 0, 0.0, 0);
                    robot.launcherBlocker.cmdBlock();
                    // currentStage = stage._50_Pickup1;
                    currentStage = stage._35_Drivetunnel;
                }

                break;
            case _35_Drivetunnel:
                if (!follower.isBusy()) {
                    follower.followPath(Tunnelpickupa);
                    currentStage = stage._37_DriveBackT; // we don't need to do the turn since heading is adjusted in path
                    runtime.reset();
                }
                break;
            case _37_DriveBackT:
                if (!follower.isBusy()) {
                    if (runtime.milliseconds() >= 500) {
                        telemetryMU.addLine("wqiting to shoot 1");
                        // if (CommonLogic.inRange(follower.getPose().getX(), wallScoreX, xTol) &&
                        //         CommonLogic.inRange(follower.getPose().getY(), wallScoreY, yTol)) {
                    robot.launcher.cmdOutfar();
                        runtime.reset();
                        currentStage = stage._45_Launch;
                    }}
                break;
            case _45_Launch:
                if (runtime.milliseconds() >= 1500) {
                    // robot.driveTrain.CmdDrive(0, 0, 0.0, 0);
              dolaunch_process();
                    // currentStage = stage._50_Pickup1;
                    currentStage = stage._50_DriveCornor;
                }

                break;
            case _50_DriveCornor:
                if (!follower.isBusy()) {
                    follower.followPath(PickupCorner1);
                    endlaunch_process();
                    currentStage = stage._55_DriveBackC; // we don't need to do the turn since heading is adjusted in path
                    runtime.reset();
                }
                break;
            case _55_DriveBackC:
                if (!follower.isBusy()) {
                    if (runtime.milliseconds() >= 500) {
                        telemetryMU.addLine("wqiting to shoot 1");
                        // if (CommonLogic.inRange(follower.getPose().getX(), wallScoreX, xTol) &&
                        //         CommonLogic.inRange(follower.getPose().getY(), wallScoreY, yTol)) {
                        robot.launcher.cmdOutfar();
                        runtime.reset();
                        currentStage = stage._60_Shoot;
                    }}
                break;
            case _60_Shoot:
                if (runtime.milliseconds() >= 1500) {
               dolaunch_process();
                    currentStage = stage._65_Drivetunnel;
                }
                break;
            case _65_Drivetunnel:
                if (!follower.isBusy()) {
                    follower.followPath(Tunnelpickupa);
                    endlaunch_process();
                    currentStage = stage._67_DriveBackT; // we don't need to do the turn since heading is adjusted in path
                    runtime.reset();
                }
                break;
            case _67_DriveBackT:
                if (!follower.isBusy()) {
                    if (runtime.milliseconds() >= 500) {
                        telemetryMU.addLine("wqiting to shoot 1");
                        // if (CommonLogic.inRange(follower.getPose().getX(), wallScoreX, xTol) &&
                        //         CommonLogic.inRange(follower.getPose().getY(), wallScoreY, yTol)) {
                        robot.launcher.cmdOutfar();
                        runtime.reset();
                        currentStage = stage._70_Launch;
                    }}
                break;
            case _70_Launch:
                if (runtime.milliseconds() >= 1500) {
                    // robot.driveTrain.CmdDrive(0, 0, 0.0, 0);
                  dolaunch_process();
                    // currentStage = stage._50_Pickup1;
                    currentStage = stage._75_DriveCornor;
                }

                break;

            case _75_DriveCornor:
                if (!follower.isBusy()) {
                    follower.followPath(PickupCorner1);
                    endlaunch_process();
                    currentStage = stage._80_DriveBackC; // we don't need to do the turn since heading is adjusted in path
                    runtime.reset();
                }
                break;
            case _80_DriveBackC:
                if (!follower.isBusy()) {
                    if (runtime.milliseconds() >= 500) {
                        telemetryMU.addLine("wqiting to shoot 1");
                        // if (CommonLogic.inRange(follower.getPose().getX(), wallScoreX, xTol) &&
                        //         CommonLogic.inRange(follower.getPose().getY(), wallScoreY, yTol)) {
                        robot.launcher.cmdOutfar();
                        runtime.reset();
                        currentStage = stage._85_Shoot;
                    }}
                break;
            case _85_Shoot:
                if (runtime.milliseconds() >= 1500) {
                    dolaunch_process();
                    currentStage = stage._90_Drivetunnel;
                }
                break;
            case _90_Drivetunnel:
                if (!follower.isBusy()) {
                    follower.followPath(Tunnelpickupa);
                    endlaunch_process();
                    currentStage = stage._100_Prelaunch; // we don't need to do the turn since heading is adjusted in path
                    runtime.reset();
                }
                break;
            case _100_Prelaunch:
                if (!follower.isBusy()) {
                    if (runtime.milliseconds() >= 500) {
                        telemetryMU.addLine("wqiting to shoot 1");
                        // if (CommonLogic.inRange(follower.getPose().getX(), wallScoreX, xTol) &&
                        //         CommonLogic.inRange(follower.getPose().getY(), wallScoreY, yTol)) {
                        robot.launcher.cmdOutfar();
                        runtime.reset();
                        currentStage = stage._110_Launch;
                    }}
                break;
            case _110_Launch:
                if (runtime.milliseconds() >= 1500) {
                    // robot.driveTrain.CmdDrive(0, 0, 0.0, 0);
                    dolaunch_process();
                    // currentStage = stage._50_Pickup1;
                    currentStage = stage._115_DriveCornor;
                }

                break;

            case _115_DriveCornor:
                if (!follower.isBusy()) {
                    follower.followPath(PickupCorner1);
                    endlaunch_process();
                    currentStage = stage._120_DriveBackc; // we don't need to do the turn since heading is adjusted in path
                    runtime.reset();
                }
                break;
            case _120_DriveBackc:
                if (!follower.isBusy()) {
                    if (runtime.milliseconds() >= 500) {
                        telemetryMU.addLine("wqiting to shoot 1");
                        // if (CommonLogic.inRange(follower.getPose().getX(), wallScoreX, xTol) &&
                        //         CommonLogic.inRange(follower.getPose().getY(), wallScoreY, yTol)) {
                        robot.launcher.cmdOutfar();
                        runtime.reset();
                        currentStage = stage._130_Shoot;
                    }}
                break;
            case _130_Shoot:
                if (runtime.milliseconds() >= 1500) {
                    // robot.driveTrain.CmdDrive(0, 0, 0.0, 0);
                    dolaunch_process();
                    // currentStage = stage._50_Pickup1;
                    currentStage = stage._135_Drivetunnel;
                }

                break;
            case _135_Drivetunnel:
                if (!follower.isBusy()) {
                    follower.followPath(Tunnelpickupa);
                    endlaunch_process();
                    currentStage = stage._140_DriveBackT; // we don't need to do the turn since heading is adjusted in path
                    runtime.reset();
                }
                break;
            case _140_DriveBackT:
                if (!follower.isBusy()) {
                    if (runtime.milliseconds() >= 500) {
                        telemetryMU.addLine("wqiting to shoot 1");
                        // if (CommonLogic.inRange(follower.getPose().getX(), wallScoreX, xTol) &&
                        //         CommonLogic.inRange(follower.getPose().getY(), wallScoreY, yTol)) {
                        robot.launcher.cmdOutfar();
                        runtime.reset();
                        currentStage = stage._145_Launch;
                    }}
                break;
            case _145_Launch:
                if (runtime.milliseconds() >= 1500) {
                    // robot.driveTrain.CmdDrive(0, 0, 0.0, 0);
                    dolaunch_process();
                    // currentStage = stage._50_Pickup1;
                    currentStage = stage._150_DriveCornor;
                }

                break;
            case _150_DriveCornor:
                if (!follower.isBusy()) {
                    follower.followPath(PickupCorner1);
                    endlaunch_process();
                    currentStage = stage._155_DriveBackC; // we don't need to do the turn since heading is adjusted in path
                    runtime.reset();
                }
                break;
            case _155_DriveBackC:
                if (!follower.isBusy()) {
                    if (runtime.milliseconds() >= 500) {
                        telemetryMU.addLine("wqiting to shoot 1");
                        // if (CommonLogic.inRange(follower.getPose().getX(), wallScoreX, xTol) &&
                        //         CommonLogic.inRange(follower.getPose().getY(), wallScoreY, yTol)) {
                        robot.launcher.cmdOutfar();
                        runtime.reset();
                        currentStage = stage._160_Shoot;
                    }}
                break;
            case _160_Shoot:
                if (runtime.milliseconds() >= 1500) {
                    // robot.driveTrain.CmdDrive(0, 0, 0.0, 0);
                    dolaunch_process();
                    // currentStage = stage._50_Pickup1;
                    currentStage = stage._165_Drivetunnel;
                }

                break;
            case _165_Drivetunnel:
                if (!follower.isBusy()) {
                    follower.followPath(Tunnelpickupa);
                    endlaunch_process();
                    currentStage = stage._167_DriveBackT; // we don't need to do the turn since heading is adjusted in path
                    runtime.reset();
                }
                break;
            case _167_DriveBackT:
                if (!follower.isBusy()) {
                    if (runtime.milliseconds() >= 500) {
                        telemetryMU.addLine("wqiting to shoot 1");
                        // if (CommonLogic.inRange(follower.getPose().getX(), wallScoreX, xTol) &&
                        //         CommonLogic.inRange(follower.getPose().getY(), wallScoreY, yTol)) {
                        robot.launcher.cmdOutfar();
                        runtime.reset();
                        currentStage = stage._170_Launch;
                    }}
                break;
            case _170_Launch:
                if (runtime.milliseconds() >= 1500) {
                    // robot.driveTrain.CmdDrive(0, 0, 0.0, 0);
                    dolaunch_process();
                    // currentStage = stage._50_Pickup1;
                    currentStage = stage._175_DriveCornor;
                }

                break;
            case _175_DriveCornor:
                if (!follower.isBusy()) {
                    follower.followPath(PickupCorner1);
                    endlaunch_process();
                    currentStage = stage._180_DriveBackC; // we don't need to do the turn since heading is adjusted in path
                    runtime.reset();
                }


                break;
            case _180_DriveBackC:
                if (!follower.isBusy()) {
                    if (runtime.milliseconds() >= 500) {
                        telemetryMU.addLine("wqiting to shoot 1");
                        // if (CommonLogic.inRange(follower.getPose().getX(), wallScoreX, xTol) &&
                        //         CommonLogic.inRange(follower.getPose().getY(), wallScoreY, yTol)) {
                        robot.launcher.cmdOutfar();
                        runtime.reset();
                        currentStage = stage._185_Shoot;
                    }}
                break;

            case _185_Shoot:
                if (!follower.isBusy()) {
                    // robot.driveTrain.CmdDrive(0, 0, 0.0, 0);
                    dolaunch_process();
                    // currentStage = stage._50_Pickup1;
                    currentStage = stage._200_DrivePark;
                }

                break;
            case _200_DrivePark:
                if (runtime.milliseconds() >= 1250) {
                    endlaunch_process();
                    follower.followPath(ParkPath,true);
                    runtime.reset();
                    currentStage = stage._500_End;
                }

                break;

            case _500_End:
            { //do nothing let the time run out
                if (runtime.milliseconds() > 3000){ //was 2500
                    follower.breakFollowing();
                }
            }




            break;
        }


        updateTelemetry();
    }  //  loop


    private void updateTelemetry() {
        telemetryMU.addData("Follower Busy?", follower.isBusy());
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
        Drawing.drawDebug(follower);
    }
    private void dolaunch_process(){

        robot.launcherBlocker.cmdUnBlock();
        robot.transitionRoller.cmdSpin();
        robot.intake.cmdFoward();
        runtime.reset();

    }

    private void endlaunch_process(){

        robot.launcherBlocker.cmdBlock();
        robot.autoRPM.Measure = false;
        robot.launcher.cmdStop();

    }


    //Code to run ONCE after the driver hits STOP


    @Override
    public void stop() {
        robot.stop();
    }


    private enum stage {
        _unknown,
        _00_preStart,
        _10_Prelaunch,
        _15_Launch,
        _16_Spike,
        _17_drivebacK,
        _20_DriveCornor,
        _25_DriveBack,
        _27_Prelaunch,
        _30_Shoot,
        _35_Drivetunnel,
        _37_DriveBackT,
        _40_PreLaunch,
        _45_Launch,
        _50_DriveCornor,
        _55_DriveBackC,
        _60_Shoot,
        _65_Drivetunnel,
        _67_DriveBackT,
        _70_Launch,
        _75_DriveCornor,
        _80_DriveBackC,
        _85_Shoot,
        _90_Drivetunnel,
        _100_Prelaunch,
        _110_Launch,
        _115_DriveCornor,
        _120_DriveBackc,
        _130_Shoot,
        _135_Drivetunnel,
        _140_DriveBackT,
        _145_Launch,
        _150_DriveCornor,
        _155_DriveBackC,
        _160_Shoot,
        _165_Drivetunnel,
        _167_DriveBackT,
        _170_Launch,
        _175_DriveCornor,
        _180_DriveBackC,
        _185_Shoot,
        _190_Drivetunnel,
        _200_DrivePark,
        _500_End


    }


}