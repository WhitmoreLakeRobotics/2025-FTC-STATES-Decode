package org.firstinspires.ftc.teamcode.pedroPathing.fastWrite;

import static org.firstinspires.ftc.teamcode.pedroPathing.CompBotConstants.pathConstraints;

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

import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.CompBotConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.Drawing;

@Disabled
@Autonomous(name = "FWRedNearattempt", group = "FW")
public class FWRedNearattempt extends OpMode {

    Robot robot = new Robot();

    private String thisUpdate = "0";
    private TelemetryManager telemetryMU;
    public String Alliance = "RED"; // Red or Blue
    public String Grounds ="NEAR"; // Near or Far
    public int Cycles = 4; // amount of cycles,(goes up to 6)
    public boolean Park = true; // if true, will park off line if it meets criteria
    public int Gates = 111100; // X = times 10 unless over 100,000 >and< B = set to zero
    // 0 = do no gate, 1 = do gate, first pos = first pickup, second pos = second pickup... etc

    public int Wraps = 1100; // X = times 10 unless over 100,000 >and< B = set to zero
    public int C1 = 1; // 0 = no position, 1 = first spike from near position,
    // 2 = 2nd, 3 = 3rd, 4 = HumanPlayerZone, 5 = DeepHumanPlayerZone
    public int C2 = 2;
    public int C3 = 0; //maybe 1
    public int C4 = 0; //maybe 1
    public int C5 = 0;
    public int C6 = 0;

    //Unknown Variables
    private int CyclesRemaining = 0;
    private double currentCode1 = 0.0;
    private double currentCode2 = 0.0;
    private double currentCode3 = 0.0;
    private double currentCode4 = 0.0;
    private double currentCode5 = 0.0;
    private double currentCode6 = 0.0;
    private boolean doGate = false;
    private boolean doWrap = false;
    private boolean isBottom = false;
    private boolean launched = false;


    //public int Cursor = 1;
    // private int CurrentTrigger = 1;

    private stage currentStage = stage._00_unknown;
    private trigger currentTrigger = trigger.Alliance;
    private codePos currentCodePos = codePos.C1;
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime gametime = new ElapsedTime();

    public static Follower follower;
    public Pose currentPose = new Pose(0,0,0);
    public static Pose startPose = new Pose(10, 10, Math.toRadians(90)); // Start Pose of our robot.
    public static Pose scorePose = new Pose(15, 15, Math.toRadians(114)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    //private final Pose scorePose = new Pose(wallScoreX, wallScoreY, wallScoreH); // seeing if configurables work for this. Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    public static Pose scorePoseAP = new Pose(20, 20, Math.toRadians(10));
    public static Pose pickup1aPose = new Pose(25, 25, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    public static Pose pickup1bPose = new Pose(20, 20, Math.toRadians(190)); // (First Set) of Artifacts picked up.
    public static Pose pickup1bPoseC = new Pose(1, 27, Math.toRadians(200));
    public static Pose pickup1cPose = new Pose(4, 13.5, Math.toRadians(180));
    public static Pose pickTunnelBlue = new Pose(125, 35, Math.toRadians(-10));
    public static Pose scorePose2 = new Pose(72, 96, Math.toRadians(10));
    public static Pose scoreCheck = new Pose(90,135,(Math.toRadians(90)));   //check
    public static Pose startPose2 = new Pose(110, 135, Math.toRadians(90));
    //  public static Pose scoreCheckCorrect = new Pose (54,135, Math.toRadians(-90));//check
    public static Pose spikeB1start = new Pose (35,84,Math.toRadians(180));
    public static Pose spikeB1end = new Pose (15,84,Math.toRadians(180));
    public static Pose spikeB2start = new Pose (35,60,Math.toRadians(180));
    public static Pose spikeB2end = new Pose (15,60,Math.toRadians(180));
    public static Pose spikeB3start = new Pose (35,36,Math.toRadians(180));
    public static Pose spikeB3end = new Pose (15,36,Math.toRadians(180));
    public static Pose spikeR1start = new Pose (110,84,Math.toRadians(0));
    public static Pose spikeR1end = new Pose (130,84,Math.toRadians(0));
    public static Pose spikeR2start = new Pose (110,60,Math.toRadians(0));
    public static Pose spikeR2end = new Pose (130,60,Math.toRadians(0));
    public static Pose spikeR3start = new Pose (110,36,Math.toRadians(0));
    public static Pose spikeR3end = new Pose (130,36,Math.toRadians(0));
    public static Pose launchFarRed = new Pose(48, 96, Math.toRadians(135));
    public static Pose launchFarBlue = new Pose(96, 96, Math.toRadians(45));
    public static Pose pickTunnelRed = new Pose(19, 35, Math.toRadians(190));
    public static Pose LaunchRN = new Pose(60,84,Math.toRadians(-45)); // -45 is placeholder
    public static Pose LaunchBN = new Pose(84,84,Math.toRadians(45)); // 45 is placeholder
    public static Pose ParkRN = new Pose(94,118,Math.toRadians(25));
    public static Pose ParkRF = new Pose(127,8,Math.toRadians(0));
    public static Pose ParkBN = new Pose(14,8,Math.toRadians(180));
    public static Pose ParkBF = new Pose(48,118,Math.toRadians(150));
    public static Pose ControlCenter = new Pose(72,68,Math.toRadians(0));
    public static Pose ControlOutpostB = new Pose(57,42,Math.toRadians(0));
    public static Pose ControlOutpostR = new Pose(87,42,Math.toRadians(0));
    public static Pose scoreBN = new Pose(55,86,Math.toRadians(135));
    public static Pose scoreBF = new Pose(59,11,Math.toRadians(114));
    public static Pose scoreRN = new Pose(89,86,Math.toRadians(50));
    public static Pose scoreRF = new Pose(85,11,Math.toRadians(70));
    public static Pose GateB = new Pose(15.5,70.5,Math.toRadians(180));
    public static Pose GateBconT = new Pose(28,72,Math.toRadians(0));
    public static Pose GateBconB = new Pose(31,61,Math.toRadians(0));
    public static Pose GateR = new Pose(126,71,Math.toRadians(0));
    public static Pose GateRconT = new Pose(112,73,Math.toRadians(0));
    public static Pose GateRconB = new Pose(116,59,Math.toRadians(0));
    public static Pose wrapAroundR = new Pose(130,59,Math.toRadians(60));
    public static Pose wrapAroundB = new Pose(12,59,Math.toRadians(120));
    public static Pose HPPickupB = new Pose(11,10,Math.toRadians(190));
    public static Pose HPPickupR = new Pose(130,11,Math.toRadians(-10));
    public static Pose TunnelB = new Pose(13,24,Math.toRadians(180));
    public static Pose TunnelR = new Pose(129,24,Math.toRadians(0));

    //start poses
    public static Pose StartRN = new Pose(33.5,134,Math.toRadians(0)).mirror(); // make better
    public static Pose StartRF = new Pose(55,8,Math.toRadians(90)).mirror(); // make better
    public static Pose StartBN = new Pose(33.5,134,Math.toRadians(180));
    public static Pose StartBF = new Pose(55,8,Math.toRadians(90));

    private PathChain scorePreload;
    private PathChain grabPickup1, grabPickup1a, grabPickup1b, grabPickup1c, scorePickup1,
            grabPickup2a, grabPickup2b, scorePickup2, goEndPose, goEndPose2, endPath;
    private PathChain cyclePickup1,spikeB1, spikeB2, spikeB3, spikeR1, spikeR2, spikeR3,
            doLaunchRN, doLaunchBN, doLaunchRF, doLaunchBF, doPickupBT ,doPickupRT,ParkREDNEAR,
            ParkREDFAR,ParkBLUENEAR,ParkBLUEFAR,ScoreRN,ScoreBN,ScoreRF,ScoreBF,PreScoreRF,
            PreScoreRN,PreScoreBN,PreScoreBF,GateBT,GateRT,GateBB,GateRB,WrapR,WrapB,HPZR,HPZB,TunPicR,
            TunPicB;

    public void buildPaths() {
        cyclePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1aPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1aPose.getHeading())

                .addPath(new BezierLine(pickup1aPose, pickup1bPose))
                .setLinearHeadingInterpolation(pickup1aPose.getHeading(), pickup1bPose.getHeading())

                .addPath(new BezierLine(pickup1bPose, scorePoseAP))
                .setLinearHeadingInterpolation(pickup1bPose.getHeading(), scorePose.getHeading())
                .build();

        spikeB1 = follower.pathBuilder()
                .addPath(new BezierLine(currentPose, spikeB1start))
                .setLinearHeadingInterpolation(currentPose.getHeading(), spikeB1start.getHeading())

                .addPath (new BezierLine(spikeB1start,spikeB1end))
                .setLinearHeadingInterpolation(spikeB1start.getHeading(), spikeB1end.getHeading())
                .build();

        spikeB2 = follower.pathBuilder()
                .addPath(new BezierLine(currentPose, spikeB2start))
                .setLinearHeadingInterpolation(currentPose.getHeading(), spikeB2start.getHeading())

                .addPath (new BezierLine(spikeB2start,spikeB2end))
                .setLinearHeadingInterpolation(spikeB2start.getHeading(), spikeB2end.getHeading())
                .build();

        spikeB3 = follower.pathBuilder()
                .addPath(new BezierLine(currentPose, spikeB3start))
                .setLinearHeadingInterpolation(currentPose.getHeading(), spikeB3start.getHeading())

                .addPath (new BezierLine(spikeB3start,spikeB3end))
                .setLinearHeadingInterpolation(spikeB3start.getHeading(), spikeB3end.getHeading())
                .build();

        spikeR1 = follower.pathBuilder()
                .addPath(new BezierLine(currentPose, spikeR1start))
                .setLinearHeadingInterpolation(currentPose.getHeading(), spikeR1start.getHeading())

                .addPath (new BezierLine(spikeR1start,spikeR1end))
                .setLinearHeadingInterpolation(spikeR1start.getHeading(), spikeR1end.getHeading())
                .build();

        spikeR2 = follower.pathBuilder()
                .addPath(new BezierLine(currentPose, spikeR2start))
                .setLinearHeadingInterpolation(currentPose.getHeading(), spikeR2start.getHeading())

                .addPath (new BezierLine(spikeR2start,spikeR2end))
                .setLinearHeadingInterpolation(spikeR2start.getHeading(), spikeR2end.getHeading())
                .build();

        spikeR3 = follower.pathBuilder()
                .addPath(new BezierLine(currentPose, spikeR3start))
                .setLinearHeadingInterpolation(currentPose.getHeading(), spikeR3start.getHeading())

                .addPath (new BezierLine(spikeR3start,spikeR3end))
                .setLinearHeadingInterpolation(spikeR3start.getHeading(), spikeR3end.getHeading())
                .build();
/*
       doLaunchRN = follower.pathBuilder()
               .addPath (new BezierLine(currentPose,LaunchRN))
               .setLinearHeadingInterpolation(currentPose.getHeading(), LaunchRN.getHeading())
               .build();

       doLaunchBN = follower.pathBuilder()
               .addPath (new BezierLine(currentPose,LaunchBN))
               .setLinearHeadingInterpolation(currentPose.getHeading(), LaunchBN.getHeading())
               .build();
\
       doLaunchRF = follower.pathBuilder()
               .addPath (new BezierLine(currentPose,launchFarRed))
               .setLinearHeadingInterpolation(currentPose.getHeading(), launchFarRed.getHeading())
               .build();

       doLaunchBF = follower.pathBuilder()
               .addPath (new BezierLine(currentPose,launchFarBlue))
               .setLinearHeadingInterpolation(currentPose.getHeading(), launchFarBlue.getHeading())
               .build();
*/
        doPickupBT = follower.pathBuilder()
                .addPath (new BezierLine(currentPose,pickTunnelBlue))
                .setLinearHeadingInterpolation(currentPose.getHeading(), pickTunnelBlue.getHeading())
                .build();

        doPickupRT = follower.pathBuilder()
                .addPath (new BezierLine(currentPose,pickTunnelRed))
                .setLinearHeadingInterpolation(currentPose.getHeading(), pickTunnelRed.getHeading())
                .build();

        ParkREDNEAR = follower.pathBuilder()
                .addPath (new BezierLine(currentPose,ParkRN))
                .setLinearHeadingInterpolation(currentPose.getHeading(), ParkRN.getHeading())
                .build();
        ParkREDFAR = follower.pathBuilder()
                .addPath (new BezierLine(currentPose,ParkRF))
                .setLinearHeadingInterpolation(currentPose.getHeading(), ParkRF.getHeading())
                .build();
        ParkBLUENEAR = follower.pathBuilder()
                .addPath (new BezierLine(currentPose,ParkBN))
                .setLinearHeadingInterpolation(currentPose.getHeading(), ParkBN.getHeading())
                .build();
        ParkBLUEFAR = follower.pathBuilder()
                .addPath (new BezierLine(currentPose,ParkBF))
                .setLinearHeadingInterpolation(currentPose.getHeading(), ParkBF.getHeading())
                .build();

        ScoreRN = follower.pathBuilder()
                .addPath (new BezierCurve(currentPose,ControlCenter,scoreRN))
                .setLinearHeadingInterpolation(currentPose.getHeading(), scoreRN.getHeading())
                .build();
        ScoreBN = follower.pathBuilder()
                .addPath (new BezierCurve(currentPose,ControlCenter,scoreBN))
                .setLinearHeadingInterpolation(currentPose.getHeading(), scoreBN.getHeading())
                .build();
        ScoreRF = follower.pathBuilder()
                .addPath (new BezierCurve(currentPose,ControlOutpostR,scoreRF))
                .setLinearHeadingInterpolation(currentPose.getHeading(), scoreRF.getHeading())
                .build();
        ScoreBF = follower.pathBuilder()
                .addPath (new BezierCurve(currentPose,ControlOutpostB,scoreBF))
                .setLinearHeadingInterpolation(currentPose.getHeading(), scoreBF.getHeading())
                .build();

        PreScoreRN = follower.pathBuilder()
                .addPath (new BezierLine(currentPose,scoreRN)) // could be start pos
                .setLinearHeadingInterpolation(currentPose.getHeading(), scoreRN.getHeading())
                .build();
        PreScoreBN = follower.pathBuilder()
                .addPath (new BezierLine(currentPose,scoreBN))
                .setLinearHeadingInterpolation(currentPose.getHeading(), scoreBN.getHeading())
                .build();
        PreScoreRF = follower.pathBuilder()
                .addPath (new BezierLine(currentPose,scoreRF))
                .setLinearHeadingInterpolation(currentPose.getHeading(), scoreRF.getHeading())
                .build();
        PreScoreBF = follower.pathBuilder()
                .addPath (new BezierLine(currentPose,scoreBF))
                .setLinearHeadingInterpolation(currentPose.getHeading(), scoreBF.getHeading())
                .build();

        GateBT = follower.pathBuilder()
                .addPath (new BezierCurve(currentPose,GateBconT,GateB))
                .setLinearHeadingInterpolation(currentPose.getHeading(), GateB.getHeading())
                .build();
        GateBB = follower.pathBuilder()
                .addPath (new BezierCurve(currentPose,GateBconB,GateB))
                .setLinearHeadingInterpolation(currentPose.getHeading(), GateB.getHeading())
                .build();
        GateRT = follower.pathBuilder()
                .addPath (new BezierCurve(currentPose,GateRconT,GateR))
                .setLinearHeadingInterpolation(currentPose.getHeading(), GateR.getHeading())
                .build();
        GateRB = follower.pathBuilder()
                .addPath (new BezierCurve(currentPose,GateRconB,GateR))
                .setLinearHeadingInterpolation(currentPose.getHeading(), GateR.getHeading())
                .build();

        WrapR = follower.pathBuilder()
                .addPath (new BezierLine(currentPose,wrapAroundR))
                .setLinearHeadingInterpolation(currentPose.getHeading(), wrapAroundR.getHeading())
                .build();
        WrapB = follower.pathBuilder()
                .addPath (new BezierLine(currentPose,wrapAroundB))
                .setLinearHeadingInterpolation(currentPose.getHeading(), wrapAroundB.getHeading())
                .build();

        HPZR = follower.pathBuilder()
                .addPath (new BezierLine(currentPose,HPPickupR))
                .setLinearHeadingInterpolation(currentPose.getHeading(), HPPickupR.getHeading())
                .build();
        HPZB = follower.pathBuilder()
                .addPath (new BezierLine(currentPose,HPPickupB))
                .setLinearHeadingInterpolation(currentPose.getHeading(), HPPickupB.getHeading())
                .build();

        TunPicR = follower.pathBuilder()
                .addPath (new BezierLine(currentPose,TunnelR))
                .setLinearHeadingInterpolation(currentPose.getHeading(), TunnelR.getHeading())
                .build();
        TunPicB = follower.pathBuilder()
                .addPath (new BezierLine(currentPose,TunnelB))
                .setLinearHeadingInterpolation(currentPose.getHeading(), TunnelB.getHeading())
                .build();
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
        Drawing.init();
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
/* maybe use
       telemetry.addData("Alliance",Alliance);
       telemetry.addData("Grounds",Grounds);
       telemetry.addData("Cycles",Cycles);
       telemetry.addData("Park",Park);
       telemetry.addData("Gates",Gates);
       telemetry.addData("Wraps",Wraps);
       telemetry.addData("C1",C1);
       telemetry.addData("C2",C2);
       telemetry.addData("C3",C3);
       telemetry.addData("C4",C4);
       telemetry.addData("C5",C5);
       telemetry.addData("C6",C6);
*/
        //super.init_loop();
        robot.init_loop();
    }

    @Override
    public void start () {
        //super.start();
        robot.start();
        gametime.reset();
        if(Alliance == "RED"){
            if(Grounds == "NEAR"){
                follower.setStartingPose(StartRN);
            }else{
                follower.setStartingPose(StartRF);
            }
        }else{
            if(Grounds == "NEAR"){
                follower.setStartingPose(StartBN);
            }else{
                follower.setStartingPose(StartBF);
            }
        }
        scanDigitCode(Gates,true);
        scanDigitCode(Wraps,false);
    }

    @Override
    public void loop() {
        setCurrentPose();
        updateTelemetry();
        telemetry.addData("Auton_Current_Stage ", currentStage);
        robot.autonLoop();
        follower.update();
        switch (currentStage) {
            case _00_unknown:
                currentStage = stage._10_preStart;
                break;

            case _10_preStart:
                CyclesRemaining = Cycles;
                currentStage = stage._20_PreloadPrelaunch;
                break;

            case _20_PreloadPrelaunch:
                if(Alliance == "RED"){
                    if(Grounds == "NEAR"){
                        follower.followPath(PreScoreRN);
                        StartLauncher();
                        currentStage = stage._30_PreloadLaunch;
                    }else{
                        follower.followPath(PreScoreRF);
                        StartLauncher();
                        currentStage = stage._30_PreloadLaunch;
                    }
                }else{
                    if(Grounds == "NEAR"){
                        follower.followPath(PreScoreBN);
                        StartLauncher();
                        currentStage = stage._30_PreloadLaunch;
                    }else{
                        follower.followPath(PreScoreBF);
                        StartLauncher();
                        currentStage = stage._30_PreloadLaunch;
                    }
                }
                //follower.followPath(scorePreload); // change or check // depend on booleans

                break;

            case _30_PreloadLaunch:
                if (!follower.isBusy()) {
                    dolaunch_process();
                    runtime.reset();
                    telemetryMU.addData("Corner pickup", follower.getPose());
                    currentStage = stage._40_Pickup;
                }
                break;
            case _40_Pickup:
                if (runtime.milliseconds() >= 1000 || robot.sensors.Empty) { // change time // if empty
                    if(!launched){
                        endlaunch_process();  // run once
                        launched = true;
                    }

                    if(CyclesRemaining > 0) {
                        if (Cycles == CyclesRemaining) {
                            if(scanDoubleCode(currentCode1,true) == 1){
                                currentStage = stage._90_Gate;
                            }
                            if (scanDoubleCode(currentCode1,false) == 1){
                                currentStage = stage._90_Gate;
                            }else{
                                currentStage = stage._50_PreLaunch;
                            }
                            CyclesRemaining = CyclesRemaining - 1;
                            scanPickup(C1);
                        } else if (Cycles == CyclesRemaining + 1) {
                            if(scanDoubleCode(currentCode2,true) == 1){
                                currentStage = stage._90_Gate;
                            }
                            if (scanDoubleCode(currentCode2,false) == 1){
                                currentStage = stage._90_Gate;
                            }else{
                                currentStage = stage._50_PreLaunch;
                            }
                            CyclesRemaining = CyclesRemaining - 1;
                            scanPickup(C2);
                        } else if (Cycles == CyclesRemaining + 2) {
                            if(scanDoubleCode(currentCode3,true) == 1){
                                currentStage = stage._90_Gate;
                            }
                            if (scanDoubleCode(currentCode3,false) == 1){
                                currentStage = stage._90_Gate;
                            }else{
                                currentStage = stage._50_PreLaunch;
                            }
                            CyclesRemaining = CyclesRemaining - 1;
                            scanPickup(C3);
                        }else if(Cycles == CyclesRemaining + 3){
                            if(scanDoubleCode(currentCode4,true) == 1){
                                currentStage = stage._90_Gate;
                            }
                            if (scanDoubleCode(currentCode4,false) == 1){
                                currentStage = stage._90_Gate;
                            }else{
                                currentStage = stage._50_PreLaunch;
                            }
                            CyclesRemaining = CyclesRemaining - 1;
                            scanPickup(C4);
                        }else if(Cycles == CyclesRemaining + 4){
                            if(scanDoubleCode(currentCode5,true) == 1){
                                currentStage = stage._90_Gate;
                            }
                            if (scanDoubleCode(currentCode5,false) == 1){
                                currentStage = stage._90_Gate;
                            }else{
                                currentStage = stage._50_PreLaunch;
                            }
                            CyclesRemaining = CyclesRemaining - 1;
                            scanPickup(C5);
                        }else if(Cycles == CyclesRemaining + 5){
                            if(scanDoubleCode(currentCode6,true) == 1){
                                currentStage = stage._90_Gate;
                            }
                            if (scanDoubleCode(currentCode6,false) == 1){  //was fix all here don't know if still need fixing
                                currentStage = stage._90_Gate;
                            }else{
                                currentStage = stage._50_PreLaunch;
                            }
                            if(Park) {
                                CyclesRemaining = CyclesRemaining - 1;
                            }
                            scanPickup(C6);
                        }else{
                            if(Alliance == "RED"){
                                if(Grounds == "NEAR"){
                                    follower.followPath(ParkREDNEAR);
                                }else{
                                    follower.followPath(ParkREDFAR);
                                }
                            }else{
                                if(Grounds == "NEAR"){
                                    follower.followPath(ParkBLUENEAR);
                                }else{
                                    follower.followPath(ParkBLUEFAR);
                                }
                            }
                        }
                    }else{
                        if(Alliance == "RED"){
                            if(Grounds == "NEAR"){
                                follower.followPath(ParkREDNEAR);
                            }else{
                                follower.followPath(ParkREDFAR);
                            }
                        }else{
                            if(Grounds == "NEAR"){
                                follower.followPath(ParkBLUENEAR);
                            }else{
                                follower.followPath(ParkBLUEFAR);
                            }
                        }
                    }

                    // telemetryMU.addData("Drive Complete?", follower.isBusy());
                }
                break;

            case _50_PreLaunch:
                if (!follower.isBusy() || robot.intake.autoStopped) { // add or if full
                    launched = false;
                    StartLauncher();
                    if(Alliance == "RED"){
                        if(Grounds == "NEAR"){
                            follower.followPath(ScoreRN);
                        }else{
                            follower.followPath(ScoreRF);
                        }
                    }else {
                        if (Grounds == "NEAR") {
                            follower.followPath(ScoreBN);
                        } else {
                            follower.followPath(ScoreBF);
                        }
                    }
                    //follower.followPath(scorePreload); // depend on booleans + comment this line out
                    if(gametime.milliseconds() <= 27000 || !Park) {
                        currentStage = stage._60_Launch; // only if enough time remains
                    }else{
                        currentStage = stage._70_Park;
                    }
                    // telemetryMU.addData("Drive Complete?", follower.isBusy());
                }
                break;

            case _60_Launch:
                if (!follower.isBusy()) {
                    dolaunch_process();
                    runtime.reset();
                    if(gametime.milliseconds() <= 25000 || !Park){
                        currentStage = stage._40_Pickup; // only if enough time remains
                    }
                    currentStage = stage._70_Park;
                    // telemetryMU.addData("Drive Complete?", follower.isBusy());
                }
                break;

            case _70_Park:
                if(Alliance == "RED"){
                    if(Grounds == "NEAR"){
                        follower.followPath(ParkREDNEAR);
                    }else{
                        follower.followPath(ParkREDFAR);
                    }
                }else{
                    if(Grounds == "NEAR"){
                        follower.followPath(ParkBLUENEAR);
                    }else{
                        follower.followPath(ParkBLUEFAR);
                    }
                }
                currentStage = stage._80_End;
                break;

            case _80_End:
                if(!follower.isBusy()){
                    stop();
                }
                break;

            case _90_Gate:
                if(!follower.isBusy()) {
                    if (Alliance == "RED") {
                        if(isBottom){
                            follower.followPath(GateRB);
                        }else{
                            follower.followPath(GateRT);
                        }
                    } else {
                        if(isBottom){
                            follower.followPath(GateBB);
                        }else{
                            follower.followPath(GateBT);
                        }
                    }
                    // go to red or blue gate position
                    if (doWrap) {
                        doWrap = false;
                        currentStage = stage._100_WrapAround;
                    } else {
                        currentStage = stage._50_PreLaunch;
                    }
                }
                break;

            case _100_WrapAround:
                if(!follower.isBusy()) {
                    runtime.reset();
                    if(runtime.milliseconds() >= 500) {
                        if (Alliance == "RED") {
                            follower.followPath(WrapR);
                        } else {
                            follower.followPath(WrapB);
                        }
                        currentStage = stage._50_PreLaunch;
                    }
                }
                // go to red or blue wrap around position
                break;
        }
    }

    @Override
    public void stop () {
        //super.stop();
        robot.stop();
    }

    private enum stage {
        _00_unknown,
        _10_preStart,
        _20_PreloadPrelaunch,
        _30_PreloadLaunch,
        _40_Pickup,
        _50_PreLaunch,
        _60_Launch,
        _70_Park,
        _80_End,
        _90_Gate,
        _100_WrapAround
    }

    private enum trigger{
        Alliance,
        Grounds,
        Cycles,
        Park,
        Gates,
        Wraps,
        C1,
        C2,
        C3,
        C4,
        C5,
        C6
    }

    public enum codePos {
        C1,
        C2,
        C3,
        C4,
        C5,
        C6
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
        Drawing.drawDebug(follower);
    }

    public void StartLauncher(){
        if(robot.limey.getTagID() > 0){
            robot.autoRPM.Measure = true;
        }else{
            if(Grounds == "NEAR"){
                robot.launcher.cmdOutnear();
            }else if(Grounds == "FAR"){
                robot.launcher.cmdOutfar();
            }
        }
    }

    private void dolaunch_process(){
        robot.launcher.launching = true; // temp
        robot.launcherBlocker.cmdUnBlock();
        robot.transitionRoller.cmdSpin();
        robot.intake.cmdFoward();
        runtime.reset();
    }

    private void endlaunch_process(){
        robot.launcher.launching = false; // temp
        robot.launcherBlocker.cmdBlock();
        robot.autoRPM.Measure = false;
        robot.launcher.cmdStop();
    }

    private void scanPickup(int PicPos){
        if(Alliance == "RED"){
            if(PicPos == 1){
                isBottom = false;
                follower.followPath(spikeR1);
            }else if(PicPos == 2){
                isBottom = true;
                follower.followPath(spikeR2);
            }else if(PicPos == 3){
                isBottom = true;
                follower.followPath(spikeR3);
            }else if(PicPos == 4){
                isBottom = true;
                follower.followPath(HPZR);
            }else if(PicPos == 5){
                isBottom = true;
                follower.followPath(TunPicR);
            }else{
            }
        }else{
            if(PicPos == 1){
                isBottom = false;
                follower.followPath(spikeB1);
            }else if(PicPos == 2){
                isBottom = true;
                follower.followPath(spikeB2);
            }else if(PicPos == 3){
                isBottom = true;
                follower.followPath(spikeB3);
            }else if(PicPos == 4){
                isBottom = true;
                follower.followPath(HPZB);
            }else if(PicPos == 5){
                isBottom = true;
                follower.followPath(TunPicB);
            }else{


            }
        }
    }

    public void scanDigitCode(int code, boolean isGate){
        switch (currentCodePos){
            case C1:
                if(code > 100000){
                    code = code - 100000;
                    if(isGate){
                        currentCode1 = currentCode1 + 1;
                    }else{
                        currentCode1 = currentCode1 + 0.1;
                    }
                }
                currentCodePos = codePos.C2;
                break;
            case C2:
                if(code > 10000){
                    code = code - 10000;
                    if(isGate){
                        currentCode2 = currentCode2 + 1;
                    }else{
                        currentCode2 = currentCode2 + 0.1;
                    }
                }
                currentCodePos = codePos.C3;
                break;
            case C3:
                if(code > 1000){
                    code = code - 1000;
                    if(isGate){
                        currentCode3 = currentCode3 + 1;
                    }else{
                        currentCode3 = currentCode3 + 0.1;
                    }
                }
                currentCodePos = codePos.C4;
                break;
            case C4:
                if(code > 100){
                    code = code - 100;
                    if(isGate){
                        currentCode4 = currentCode4 + 1;
                    }else{
                        currentCode4 = currentCode4 + 0.1;
                    }
                }
                currentCodePos = codePos.C5;
                break;
            case C5:
                if(code > 10){
                    code = code - 10;
                    if(isGate){
                        currentCode5 = currentCode5 + 1;
                    }else{
                        currentCode5 = currentCode5 + 0.1;
                    }
                }
                currentCodePos = codePos.C6;
                break;
            case C6:
                if(code > 1){
                    code = code - 1;
                    if(isGate){
                        currentCode6 = currentCode6 + 1;
                    }else{
                        currentCode6 = currentCode6 + 0.1;
                    }
                }
                break;
        }
    }

    public int scanDoubleCode(double dubCode, boolean isWrap){
        if(dubCode - 1.1 == 0){
            if(isWrap){
                doWrap = true;
                return 1;
            }else {
                doGate = true;
                return 1;
            }
        }else if(dubCode - 1.0 == 0){
            if(isWrap){
                doWrap = false;
                return 0;
            }else {
                doGate = true;
                return 1;
            }
        }else if(dubCode - 0.1 == 0){
            if(isWrap){
                doWrap = true;
                return 1;
            }else {
                doGate = false;
                return 0;
            }
        }else{
            if(isWrap){
                doWrap = false;
                return 0;
            }else {
                doGate = false;
                return 0;
            }
        }
    }

    public void setCurrentPose(){
        currentPose = follower.getPose();
    }

    public void nextStagePick(){
        if(!doGate) {
            if (gametime.milliseconds() <= 26000 || !Park) {
                currentStage = stage._50_PreLaunch; // only if enough time remains
            } else {
                currentStage = stage._70_Park;
            }
        }else{
            currentStage = stage._90_Gate;
        }
    }
}