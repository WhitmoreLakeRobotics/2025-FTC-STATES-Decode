package org.firstinspires.ftc.teamcode.Autons;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Common.Settings;
import org.firstinspires.ftc.teamcode.Hardware.Robot;


@Autonomous(name = "Magis25584", group = "Auton")
// @Autonomous(...) is the other common choice

public class Magis25584 extends OpMode {

    //RobotComp robot = new RobotComp();
    Robot robot = new Robot();




    private stage currentStage = stage._unknown;
    // declare auton power variables
    //private double AUTO_DRIVE_TURBO_SPEED = DriveTrain.DRIVETRAIN_TURBOSPEED;
    //private double AUTO_DRIVE_SLOW_SPEED = DriveTrain.DRIVETRAIN_SLOWSPEED;
    // private double AUTO_DRIVE_NORMAL_SPEED = DriveTrain.DRIVETRAIN_NORMALSPEED;
    // private double AUTO_TURN_SPEED = DriveTrain.DRIVETRAIN_TURNSPEED;

    private String RTAG = "8492-Auton";

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
    }


    //Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP

    @Override
    public void loop() {

        telemetry.addData("Auton_Current_Stage ", currentStage);
        robot.loop();

        switch (currentStage){
            case  _unknown:
                currentStage = stage._00_preStart;
                break;

            case _00_preStart:
                currentStage = stage._05_ForwardStart;
                break;


            case _05_ForwardStart:
                if (robot.driveTrain.getCmdComplete())     {
                    robot.driveTrain.CmdDrive(4,0,0.2,0);
                    robot.launcher.cmdOutfar();
                    runtime.reset();
                    currentStage = stage._20_Launch;
                }

/*
                break;
            case _10_PreLaunch:
                if (robot.driveTrain.getCmdComplete()) {
                    robot.driveTrain.CmdDrive(0, 0, 0.0, 0);
                    robot.launcher.cmdOutfar();
                    runtime.reset();
                    currentStage = stage._20_Launch;
                }
 */

                break;
            case _20_Launch:
                if(runtime.milliseconds() >=1000){
                    robot.driveTrain.CmdDrive(0,0,0.0,0);
                    robot.launcher.launching = true;
                    robot.launcherBlocker.cmdUnBlock();
                    robot.transitionRoller.cmdSpin();
                    robot.intake.cmdFoward();
                    runtime.reset();
                    currentStage = stage._25_StopLaunch;

                }

                break;
            case _25_StopLaunch:
                if (runtime.milliseconds() >=1500)     {
                    robot.driveTrain.CmdDrive(4,-180,0.35,-58);
                    robot.launcher.launching = false;
                    robot.launcherBlocker.cmdBlock();
                    robot.launcher.cmdStop();
                    runtime.reset();
                    currentStage = stage._30_MoveToWall;
                }

                break;
            case _30_MoveToWall:
                if (runtime.milliseconds() >= 0)     {
                    robot.driveTrain.cmdTurn(-65,0.35);
                    robot.transitionRoller.cmdSpin();
                    robot.intake.cmdFoward();
                    currentStage = stage._40_GoForward;
                }

                break;
            case _40_GoForward:
                if (robot.driveTrain.getCmdComplete())     {
                    robot.driveTrain.CmdDrive(54,-69,0.40,-92);
                    runtime.reset();
                    currentStage = stage._45_ExtraTurn;
                }

                break;
            case _45_ExtraTurn:
                if (robot.driveTrain.getCmdComplete()) {
                    robot.driveTrain.CmdDrive(10, -65, 0.40, -80);
                    robot.intake.cmdFoward();
                    runtime.reset();
                    currentStage = stage._50_MoveBackward2;

                }

                break;

            case _50_MoveBackward2:
                if (robot.driveTrain.getCmdComplete() && runtime.milliseconds() >= 1000)     {
                    robot.driveTrain.CmdDrive(47,118,0.35,-59);
                    currentStage = stage._60_TurnToShoot;
                }

                break;
            case _60_TurnToShoot:
                if (robot.driveTrain.getCmdComplete())     {
                    robot.launcher.cmdOutfar();
                    robot.driveTrain.cmdTurn(0,0.30);
                    runtime.reset();
                    currentStage = stage._65_align;
                }

                break;

            case _65_align:
                if (runtime.milliseconds() >=800) {
                    robot.driveTrain.cmdTurn((int)Math.round(robot.targetAngleCalc() -2), 0.20);
                    runtime.reset();
                    currentStage = stage._70_Launch2;
                }

                break;
            case _70_Launch2:
                if (robot.driveTrain.getCmdComplete()) {
                    if (runtime.milliseconds() >= 1500){
                        robot.driveTrain.CmdDrive(3, 0, 0.25, 0);
                        robot.launcher.launching = true;
                        robot.launcherBlocker.cmdUnBlock();
                        robot.transitionRoller.cmdSpin();
                        robot.intake.cmdFoward();
                        runtime.reset();
                        currentStage = stage._75_StopLaunch2;
                    }
                }

                break;
            case _75_StopLaunch2:
                if (runtime.milliseconds() >=2500)     {
                    robot.driveTrain.CmdDrive(4,-180,0.35,-59);
                    robot.launcher.launching = false;
                    robot.launcherBlocker.cmdBlock();
                    robot.launcher.cmdStop();
                    runtime.reset();
                    currentStage = stage._80_TurnToPickup;
                }

                break;
            case _80_TurnToPickup:
                if (runtime.milliseconds() >= 0)     {
                    robot.driveTrain.cmdTurn(-65,0.35);
                    robot.transitionRoller.cmdSpin();
                    robot.intake.cmdFoward();
                    currentStage = stage._100_Backup2;
                }

                break;
            case _100_Backup2:
                if (robot.driveTrain.getCmdComplete())     {
                    robot.driveTrain.CmdDrive(54,-68,0.40,-90);
                    runtime.reset();
                    currentStage = stage._1_Fix1;
                }

                break;
            case _1_Fix1:
                if (robot.driveTrain.getCmdComplete()) {
                    robot.driveTrain.CmdDrive(15, -65, 0.40, -80);
                    runtime.reset();
                    currentStage = stage._2_Fix2;

                }

                break;

            case _2_Fix2:
                if (robot.driveTrain.getCmdComplete() && runtime.milliseconds() >= 1000)     {
                    robot.driveTrain.CmdDrive(45,118,0.35,-58);
                    currentStage = stage._3_Fix3;
                }

                break;
            case _3_Fix3:
                if (robot.driveTrain.getCmdComplete())     {
                    robot.launcher.cmdOutfar();
                    robot.driveTrain.cmdTurn(0,0.30);
                    currentStage = stage._4_Fix4;
                }

                break;
            case _4_Fix4:
                if (robot.driveTrain.getCmdComplete())     {
                    robot.driveTrain.CmdDrive(4,0,0.25,0);
                    robot.intake.cmdFoward();
                    runtime.reset();
                    currentStage = stage._5_Fix5;
                }

                break;

            case _5_Fix5:
                if (runtime.milliseconds() >=500) {
                    robot.driveTrain.cmdTurn((int)Math.round(robot.targetAngleCalc() -2), 0.20);
                    runtime.reset();
                    currentStage = stage._103_Launch3;
                }

                /*
                break;
            case _75_StopLaunch2:
                if (runtime.milliseconds() >= 2000)     {
                    robot.driveTrain.CmdDrive(15,0,0.35,0);
                    robot.launcher.cmdStop();
                    robot.launcherBlocker.cmdBlock();
                    currentStage = stage._80_TurnToPickup;
                }

                break;
            case _80_TurnToPickup:
                if(robot.driveTrain.getCmdComplete()){
                    robot.driveTrain.cmdTurn(-58,0.30);
                    robot.transitionRoller.cmdSpin();
                    robot.intake.cmdFoward();
                    currentStage = stage._90_Pickup2;

                }

                break;
            case _90_Pickup2:
                if(robot.driveTrain.getCmdComplete()){
                    robot.driveTrain.CmdDrive(38,-65,0.40,-65);
                    runtime.reset();
                    currentStage = stage._100_Backup2;

                }

                break;
            case _100_Backup2:
                if (robot.driveTrain.getCmdComplete() && runtime.milliseconds() >= 500)     {
                    robot.driveTrain.CmdDrive(24,122,0.40,-58);
                    robot.launcher.cmdOutfar();
                    currentStage = stage._102_TurnToLaunch3;
                }

                break;
            case _102_TurnToLaunch3:
                if (robot.driveTrain.getCmdComplete())     {
                    robot.driveTrain.cmdTurn(0,0.25);
                    robot.driveTrain.CmdDrive(20,180,0.25,0);
                    currentStage = stage._103_Launch3;
                }

                 */

                break;
            case _103_Launch3:
                if (runtime.milliseconds() >= 1000)     {
                    robot.launcher.launching = true;
                    robot.launcherBlocker.cmdUnBlock();
                    robot.transitionRoller.cmdSpin();
                    robot.intake.cmdFoward();
                    runtime.reset();
                    currentStage = stage._104_TurnToBoom;
                }

                break;
            case _104_TurnToBoom:
                if(runtime.milliseconds() >= 1500){
                    robot.launcher.launching = false;
                    robot.launcherBlocker.cmdBlock();
                    robot.transitionRoller.cmdSpin();
                    robot.intake.cmdFoward();
                    robot.driveTrain.cmdTurn(-58,0.30);
                    runtime.reset();
                    currentStage = stage._105_Boom;

                }


                break;
            case _105_Boom:
                if (robot.driveTrain.getCmdComplete())     {
                    robot.launcherBlocker.cmdBlock();
                    robot.transitionRoller.cmdSpin();
                    robot.intake.cmdFoward();
                    robot.driveTrain.CmdDrive(51,-65,0.50,-90);
                    currentStage = stage._107_ResetGyro;
                }

                break;
            case _107_ResetGyro:
                if (robot.driveTrain.getCmdComplete())     {
                    robot.driveTrain.ResetGyro();
                    currentStage = stage._110_End;
                }

                break;

            case _110_End:
                if(robot.driveTrain.getCmdComplete()){
                    robot.stop();


                }







                break;
        }



    }  //  loop


    //Code to run ONCE after the driver hits STOP

    @Override
    public void stop() {
        robot.stop();
    }

    private enum stage {
        _unknown,
        _00_preStart,
        _05_ForwardStart,
        _10_PreLaunch,
        _20_Launch,
        _25_StopLaunch,
        _30_MoveToWall,
        _40_GoForward,
        _45_ExtraTurn,
        _50_MoveBackward2,
        _60_TurnToShoot,
        _65_align,
        _70_Launch2,
        _75_StopLaunch2,
        _80_TurnToPickup,
        _90_Pickup2,
        _100_Backup2,
        _102_TurnToLaunch3,
        _1_Fix1,
        _2_Fix2,
        _3_Fix3,
        _4_Fix4,
        _5_Fix5,
        _103_Launch3,
        _104_TurnToBoom,
        _105_Boom,
        _107_ResetGyro,
        _110_End


    }
}

