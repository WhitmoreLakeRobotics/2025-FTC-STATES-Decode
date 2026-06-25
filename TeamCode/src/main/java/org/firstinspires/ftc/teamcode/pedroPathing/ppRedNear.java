package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.teamcode.pedroPathing.CompBotConstants.pathConstraints;
import static org.firstinspires.ftc.teamcode.pedroPathing.ppSharpCorner6BlueFar.TunelPose;
import static org.firstinspires.ftc.teamcode.pedroPathing.ppSharpCorner6BlueFar.poseSpike1;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Common.CommonLogic;
import org.firstinspires.ftc.teamcode.Hardware.Robot;

/******WARNING !!! This was copied from the last WOrlds code we were doing
 *****  and crashes during init with no errors
 * */

@Disabled
@Autonomous(name = "ppRedNear1", group = "PP")
public class ppRedNear extends OpMode {


    Robot robot = new Robot();

//wyatt's fail of an auton (with assistance)

    private String thisUpdate = "0";
    private TelemetryManager telemetryMU;
    private stage currentStage = stage._00_unknown;
    private ElapsedTime runtime = new ElapsedTime();
    public boolean End = false;

    public static Follower follower;
    public static Pose startPose = new Pose(108, 135, Math.toRadians(0)); // Start Pose of our robot.
    public static Pose scorePose = new Pose(108, 108, Math.toRadians(53)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    //private final Pose scorePose = new Pose(wallScoreX, wallScoreY, wallScoreH); // seeing if configurables work for this. Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    /*
    public static Pose scorePoseAP = new Pose(52, 15, Math.toRadians(10));
    public static Pose poseSpike1 = new Pose(12.5,35,Math.toRadians(210));
    public static Pose pickup1aPose = new Pose(15, 35.5, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    public static Pose pickup1bPose = new Pose(20, 20, Math.toRadians(190)); // (First Set) of Artifacts picked up.
    public static Pose pickup1bPoseC = new Pose(1, 27, Math.toRadians(200));
    public static Pose pickup1cPose = new Pose(4, 13.5, Math.toRadians(180));
    public static Pose currentPose  = new Pose(0,0,0);
    public static Pose spikeB2start = new Pose (35,60,Math.toRadians(180));
    public static Pose spikeB2end = new Pose (15,60,Math.toRadians(180));
    public static Pose CornerPickupPose = new Pose (11,11,Math.toRadians(190));
    public static Pose TunelPose = new Pose(12, 30,Math.toDegrees(165));

     */

    public static Pose spiwkep1c = new Pose(128.5,83.5,Math.toRadians(0));
    public static Pose conspiwkep1c = new Pose(74,81,Math.toRadians(0));//control
    public static Pose gatepos = new Pose(127.,70.5,Math.toRadians(0));
    public static Pose congatepos = new Pose(92,73.4,Math.toRadians(0));//control
    public static Pose wraparoundpos = new Pose(130.5,59,Math.toRadians(53));
    public static Pose conwraparoundpos = new Pose(113.5,60,Math.toRadians(0));//control
    public static Pose conaltscore = new Pose(107,68,Math.toRadians(0));//control
    public static Pose parkpos = new Pose(108,72,Math.toRadians(0));




    private PathChain scorePreload;
    private PathChain grabPickup1, grabPickup1a, grabPickup1b, grabPickup1c, scorePickup1,
            grabPickup2a, grabPickup2b, scorePickup2, goEndPose, goEndPose2, endPath;
    private PathChain interruptedPickup,
            PickupSpike1,ScorePreload,ScorePath,GateWrap,WrapScore,ParkPath;


    public void buildPaths() {

        ScorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        PickupSpike1 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, conspiwkep1c, spiwkep1c))
                .setLinearHeadingInterpolation(scorePose.getHeading(), spiwkep1c.getHeading())
                .build();

        ScorePath = follower.pathBuilder()
                .addPath(new BezierLine(spiwkep1c, scorePose))
                .setLinearHeadingInterpolation(spiwkep1c.getHeading(), scorePose.getHeading())
                .build();

        GateWrap = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, congatepos, gatepos))
                .setLinearHeadingInterpolation(scorePose.getHeading(), gatepos.getHeading())
                .addPath(new BezierCurve(gatepos, conwraparoundpos, wraparoundpos))
                .setLinearHeadingInterpolation(gatepos.getHeading(), wraparoundpos.getHeading())
                .build();

        WrapScore = follower.pathBuilder()
                .addPath(new BezierCurve(wraparoundpos, conaltscore, scorePose))
                .setLinearHeadingInterpolation(wraparoundpos.getHeading(), scorePose.getHeading())
                .build();

        ParkPath = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, parkpos))
                .setLinearHeadingInterpolation(scorePose.getHeading(), parkpos.getHeading())
                .build();

/*
        PickupSpike1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, poseSpike1))
                .setLinearHeadingInterpolation(scorePose.getHeading(), poseSpike1.getHeading())
                .build();

        cyclePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1aPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1aPose.getHeading())

                .addPath(new BezierLine(pickup1aPose, pickup1bPose))
                .setLinearHeadingInterpolation(pickup1aPose.getHeading(), pickup1bPose.getHeading())

                .addPath(new BezierCurve(pickup1bPose,pickup1bPoseC, scorePoseAP))
                .setLinearHeadingInterpolation(pickup1bPose.getHeading(), scorePose.getHeading())

                .build();
        scorePreload = follower.pathBuilder()
                .addPath (new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        spikeB2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, spikeB2start))
                .setLinearHeadingInterpolation(scorePose.getHeading(), spikeB2start.getHeading())
                .addPath (new BezierLine(spikeB2start,spikeB2end))
                .setLinearHeadingInterpolation(spikeB2start.getHeading(), spikeB2end.getHeading())
                .build();

        CornerPickup = follower.pathBuilder()
                .addPath (new BezierLine(scorePose, CornerPickupPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), CornerPickupPose.getHeading())
                .addPath (new BezierLine(CornerPickupPose, scorePose))
                .setLinearHeadingInterpolation(CornerPickupPose.getHeading(), scorePose.getHeading())
                .build();

        TunelPickup = follower.pathBuilder()
                .addPath( new BezierLine(scorePose, TunelPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), TunelPose.getHeading())
                .addPath (new BezierLine(TunelPose, scorePose))
                .setLinearHeadingInterpolation(TunelPose.getHeading(), scorePose.getHeading())
                .build();

        Park = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, CornerPickupPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), CornerPickupPose.getHeading())
                .build();
        */

    }

    @Override
    public void init() {

// NAJ CompBotConstants is a file/class that contains the definition of the gryo and drive motors among other things
        //super.init();
        follower = CompBotConstants.createFollower(hardwareMap);
        buildPaths();
        PanelsConfigurables.INSTANCE.refreshClass(this);
        follower.setStartingPose(startPose);
        follower.update();
//  pedroPanelsTelemetry.init();

        telemetryMU = PanelsTelemetry.INSTANCE.getTelemetry();

// disp[lay starting postition
        telemetryMU.addData("initialized postition - Update ", thisUpdate);
// Feedback to Driver Hub for debugging
        updateTelemetry();

        robot.hardwareMap = hardwareMap;
        robot.telemetry = telemetry;
        robot.init();
    }


    @Override
    public void init_loop() {
        //super.init_loop();


        robot.init_loop();

    }

    @Override
    public void start () {
        //super.start();
        robot.start();
    }

    @Override
    public void loop() {

        updateTelemetry();
        telemetry.addData("Auton_Current_Stage ", currentStage);
        robot.autonLoop();
        follower.update();
        switch (currentStage) {
            case _00_unknown:
                currentStage = stage._10_preStart;
                break;

            case _10_preStart:
                currentStage = stage._20_PreLaunch;
                break;

            case _20_PreLaunch:
                if (!follower.isBusy()) {
                    follower.followPath(ScorePreload,true);
                    //lastPose = startPose;
                    /* currentTargetPose = scorePose;*/
                    // follower.update();
                    robot.autoRPM.Measure = true;
                    currentStage = stage._30_ScorePreload;
                }

                break;

            case _30_ScorePreload:
                if (!follower.isBusy()) {
                    dolaunch_process();
                    currentStage = stage._40_PickupSpike1;
                }
                break;
            case _40_PickupSpike1:
                if (runtime.milliseconds() > 1500 || robot.sensors.Empty){//added sensors here
                    endlaunch_process();
                    follower.followPath(PickupSpike1,true);
                    currentStage = stage._45_PreLaunch2;
                }
                break;
            case _45_PreLaunch2:
                AreYouSure(stage._50_Launch2);

                break;
            case _50_Launch2:
                if (!follower.isBusy()) {
                    dolaunch_process();
                    currentStage = stage._60_PickupConer1;
                }
                break;
             case _60_PickupConer1:
                    if (runtime.milliseconds() > 1500 || robot.sensors.Empty){ //added sensors here
                        endlaunch_process();
                        follower.followPath(GateWrap,true);
                        runtime.reset();
                        currentStage = stage._65_GateWrap1;
                }
                break;

            case _65_GateWrap1:
                if(!follower.isBusy() && runtime.milliseconds() >= 3000){ // change timeing, maybe even how it does it
                    follower.followPath(WrapScore,true);
                    currentStage = stage._70_PreLaunch3;
                }
                break;

            case _70_PreLaunch3:
                AreYouSure(stage._75_Launch3);
                break;


            case _75_Launch3:
                if (!follower.isBusy()) {
                    dolaunch_process();
                    currentStage = stage._80_PickupTunel1;
                }
                break;
            case _80_PickupTunel1:
                if (runtime.milliseconds() > 1500 || robot.sensors.Empty){ //added sensors here
                    endlaunch_process();
                    follower.followPath(GateWrap,true);
                    runtime.reset();
                    currentStage = stage._85_GateWrap2;
                }
                break;
            case _85_GateWrap2:
                if(!follower.isBusy() && runtime.milliseconds() >= 3000){ // change timeing, maybe even how it does it
                    follower.followPath(WrapScore,true);
                    currentStage = stage._90_PreLaunch4;
                }

                break;
            case _90_PreLaunch4:
                AreYouSure(stage._100_Launch4);
                break;

            case _100_Launch4:
                if (!follower.isBusy()) {
                    dolaunch_process();
                    currentStage = stage._110_PickupCorner2;
                }
                break;
            case _110_PickupCorner2:
                if (runtime.milliseconds() > 1500 || robot.sensors.Empty){ //added sensors here
                    endlaunch_process();
                    follower.followPath(GateWrap,true);
                    runtime.reset();
                    currentStage = stage._115_GateWrap3;
                }
                break;
            case _115_GateWrap3:
                if(!follower.isBusy() && runtime.milliseconds() >= 3000){ // change timeing, maybe even how it does it
                    follower.followPath(WrapScore,true);
                    currentStage = stage._120_Prelaunch5;
                }
                break;
            case _120_Prelaunch5:
                AreYouSure(stage._130_Launch5);
                break;


            case _130_Launch5:
                if (!follower.isBusy()) {
                    dolaunch_process();
                    currentStage = stage._140_PickupTunel2;
                }
                break;
            case _140_PickupTunel2:
                if (runtime.milliseconds() > 1500 || robot.sensors.Empty){ //added sensors here
                    endlaunch_process();
                    follower.followPath(GateWrap,true);
                    runtime.reset();
                    currentStage = stage._145_GateWrap4;
                }

                break;
            case _145_GateWrap4:
                if(!follower.isBusy() && runtime.milliseconds() >= 3000){ // change timeing, maybe even how it does it
                    follower.followPath(WrapScore,true);
                    currentStage = stage._150_PreLaunch6;
                }
            case _150_PreLaunch6:
                AreYouSure(stage._160_Launch6);
                break;


            case _160_Launch6:
                if (!follower.isBusy()) {
                    dolaunch_process();
                    currentStage = stage._161_PickupCorner3;
                }
                break;


            case _161_PickupCorner3:
                if (runtime.milliseconds() > 1500 || robot.sensors.Empty){ //added sensors here
                    endlaunch_process();
                    follower.followPath(GateWrap,true);
                    currentStage = stage._1615_GateWrap5;
                }
                break;

            case _1615_GateWrap5:
                if(!follower.isBusy() && runtime.milliseconds() >= 3000){ // change timeing, maybe even how it does it
                    follower.followPath(WrapScore,true);
                    currentStage = stage._162_PreLaunch7;
                }
                break;

            case _162_PreLaunch7:
                AreYouSure(stage._163_Launch7);
                break;


            case _163_Launch7:
                if (!follower.isBusy()) {
                    dolaunch_process();
                    currentStage = stage._164_PickupTunel3;
                }
                break;


            case _164_PickupTunel3:
                if (runtime.milliseconds() > 1500 || robot.sensors.Empty){ //added sensors here
                    endlaunch_process();
                    //follower.followPath(TunelPose,true);
                    currentStage = stage._170_ParkToBeContinued;
                }
                break;
/*
            case _165_Prelaunch8:
                AreYouSure(stage._166_Launch8);
                break;

            case _166_Launch8:
                if (!follower.isBusy()) {
                    dolaunch_process();
                    currentStage = stage._170_ParkToBeContinued;
                }
                break;

 */



            case _170_ParkToBeContinued:
                if (!follower.isBusy()) {
                    follower.followPath(ParkPath);
                    currentStage = stage._200_end;
                }
                break;
            case _200_end:
                if (!follower.isBusy()) {
                    telemetryMU.addData("Drive Complete?", follower.isBusy());
                    stop();
                    runtime.reset();
                    End = true;
                }
                break;



        }

    }



    @Override
    public void stop () {
        //super.stop();

        CommonLogic.StartEndPose = follower.getPose();
        robot.stop();
    }
    private enum stage {

        _00_unknown,
        _10_preStart,
        _20_PreLaunch,
        _30_ScorePreload,
        _40_PickupSpike1,
        _42_,
        _45_PreLaunch2,
        _50_Launch2,
        _60_PickupConer1,
        _65_GateWrap1,
        _70_PreLaunch3,
        _75_Launch3,
        _80_PickupTunel1,
        _85_GateWrap2,
        _90_PreLaunch4,
        _100_Launch4,
        _110_PickupCorner2,
        _115_GateWrap3,
        _120_Prelaunch5,
        _130_Launch5,
        _140_PickupTunel2,
        _145_GateWrap4,
        _150_PreLaunch6,
        _160_Launch6,
        _161_PickupCorner3,
        _1615_GateWrap5,
        _162_PreLaunch7,
        _163_Launch7,
        _164_PickupTunel3,
        _165_Prelaunch8,
        _166_Launch8,
        _170_ParkToBeContinued,
        _200_end;


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

    private void AreYouSure(stage NextStage){

        //telemetryMU.addData("pathPose2", pickup1bPose);
        //telemetryMU.addData("scorePose", scorePoseAP);
        if (follower.isBusy()) { //we are still running path
//telemetryMU.addData("check intake status", robot.intake.AtIntakeStop); intake.AtIntakeStop is never set to false
            if (robot.intake.autoStopped) { //(robot.sensors.allFilled) */{
                telemetryMU.addLine("Intake stopped - break follower");
                // we've got 3 artifacts, stop the path and return to scorePose
                follower.breakFollowing();
                newPath();
                //   robot.autoRPM.Measure = true; // start fly wheels
                robot.autoRPM.Measure = true;
                currentStage = NextStage;
                runtime.reset();

            } else if (follower.getCurrentTValue() > 0.75) { //the path is almost done
                //  robot.autoRPM.Measure = true; //start fly wheels
                robot.autoRPM.Measure = true;
            }
        } else {// path is complete we are back at scorePose move to launch
            robot.autoRPM.Measure = true;
            currentStage = NextStage;
        }

    }

    private void updateTelemetry () {
        telemetryMU.addData("Follower Busy?", follower.isBusy());
        telemetryMU.addData("Current Stage", currentStage);
        telemetryMU.addData("x", follower.getPose().getX());
        telemetryMU.addData("y", follower.getPose().getY());
        telemetryMU.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        //  telemetryMU.addData("LAST Pose", lastPose);
        //  telemetryMU.addData("Current Target Pose", currentTargetPose);
        telemetryMU.addData("breakingStrength", pathConstraints.getBrakingStrength());
        telemetryMU.addData("breakstart ", pathConstraints.getBrakingStart());
        telemetryMU.addData("drivepid P", follower.constants.coefficientsDrivePIDF.P);
        telemetryMU.addData("drivepid D", follower.constants.coefficientsDrivePIDF.D);
        telemetryMU.addData("drivepid F", follower.constants.coefficientsDrivePIDF.F);
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

    private  void newPath(){
        interruptedPickup = follower.pathBuilder()
                .addPath (new BezierLine(follower.getPose(), scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();
        follower.followPath(interruptedPickup,true);

    }

}
