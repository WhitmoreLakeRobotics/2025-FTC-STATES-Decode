package org.firstinspires.ftc.teamcode.Autons;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Common.Settings;
import org.firstinspires.ftc.teamcode.Hardware.Robot;

@Disabled
@Autonomous(name = "BlueFarThreeCycle", group = "Auton")
// @Autonomous(...) is the other common choice

public class BlueFarThreeCycle extends OpMode {

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

            case _00_preStart:
                currentStage = stage._05_ForwardStart;
                break;


            case _05_ForwardStart:
                if (robot.driveTrain.getCmdComplete())     {
                    robot.driveTrain.CmdDrive(4,0,0.25,0);
                    robot.launcher.cmdOutfar();
                    runtime.reset();
                    currentStage = stage._20_Launch;
                }

                break;
            case _20_Launch:
                if  (runtime.milliseconds() >=1500){
                    robot.driveTrain.CmdDrive(0,0,0.0,0);
                    robot.launcherBlocker.cmdUnBlock();
                    robot.transitionRoller.cmdSpin();
                    robot.intake.cmdFoward();
                    runtime.reset();
                    currentStage = stage._25_StopLaunch;

                }

                break;
            case _25_StopLaunch:
                if (runtime.milliseconds() >=2000)     {
                    robot.driveTrain.CmdDrive(0,0,0.0,0);
                    robot.launcherBlocker.cmdBlock();
                    robot.launcher.cmdStop();
                    runtime.reset();
                    currentStage = stage._30_MoveForward;
                }

                break;
            case _30_MoveForward:
                if (robot.driveTrain.getCmdComplete())     {
                    robot.driveTrain.CmdDrive(37,5,0.35,5);
                    currentStage = stage._40_TurnLeft1;
                }

                break;
            case _40_TurnLeft1:
                if (robot.driveTrain.getCmdComplete())     {
                    robot.driveTrain.cmdTurn(-65,0.30);
                    robot.intake.cmdFoward();
                    robot.transitionRoller.cmdSpin();
                    currentStage = stage._50_MoveForward2;
                }

                break;
            case _50_MoveForward2:
                if (robot.driveTrain.getCmdComplete())     {
                    robot.driveTrain.CmdDrive(36,-65,0.35,-65);
                    currentStage = stage._60_MoveBack;
                }

                /*
                break;
            case _53_TurnToGate:
                if (robot.driveTrain.getCmdComplete())     {
                    robot.driveTrain.cmdTurn(-155,0.30);
                    currentStage = stage._57_UnturnToGate;
                }


                break;
            case _57_UnturnToGate:
                if (robot.driveTrain.getCmdComplete())     {
                    robot.driveTrain.cmdTurn(-45,0.30);
                    currentStage = stage._60_MoveBack;
                 */

                break;
            case _60_MoveBack:
                if (robot.driveTrain.getCmdComplete())     {
                    robot.driveTrain.CmdDrive(27,-245,0.35,-65); //was
                    currentStage = stage._65_UnturnCause;
                }

                break;
            case _65_UnturnCause:
                if (robot.driveTrain.getCmdComplete())     {
                    robot.driveTrain.cmdTurn(-15,0.30);
                    currentStage = stage._66_Driveback;
                }

                break;
            case _66_Driveback:
                if (robot.driveTrain.getCmdComplete())     {
                    robot.driveTrain.CmdDrive(44,180,0.35,0);
                    robot.launcher.cmdOutfar();
                    currentStage = stage._68_Alighn;
                }

                break;
            case _68_Alighn:
                if (robot.driveTrain.getCmdComplete())     {
                    runtime.reset(); //was negative
                    currentStage = stage._70_Launch2;
                }

                break;
            case _69_AutoAdjust2:
                if (runtime.milliseconds() >= 500)     {
                    robot.driveTrain.cmdTurn((int)Math.round(robot.targetAngleCalc()),0.20);
                    currentStage = stage._70_Launch2;
                }


                break;
            case _70_Launch2:
                if (robot.driveTrain.getCmdComplete())    {
                    robot.launcherBlocker.cmdUnBlock();
                    robot.transitionRoller.cmdSpin();
                    robot.intake.cmdFoward();
                    runtime.reset();
                    currentStage = stage._75_MoveForward2;
                }

                break;
            case _75_MoveForward2:
                if (runtime.milliseconds() >= 1500)   {
                    robot.launcher.cmdStop();
                    robot.launcherBlocker.cmdBlock();
                    robot.driveTrain.CmdDrive(19,0,0.35,0);
                    currentStage = stage._80_TurnToArtifact2;
                }

                break;
            case _80_TurnToArtifact2:
                if(robot.driveTrain.getCmdComplete()){
                    robot.driveTrain.cmdTurn(-65,0.30);
                    robot.intake.cmdFoward();
                    robot.transitionRoller.cmdSpin();
                    currentStage = stage._90_Forward3;

                }

                break;
            case _90_Forward3:
                if(robot.driveTrain.getCmdComplete()){
                    robot.driveTrain.CmdDrive(36,-65,0.35,-65);
                    currentStage = stage._97_backup2;
                }

                break;
            case _97_backup2:
                if(robot.driveTrain.getCmdComplete()){
                    robot.driveTrain.CmdDrive(33,-245,0.35,-65);
                    currentStage = stage._95_turn2;
                }

                break;
            case _95_turn2:
                if(robot.driveTrain.getCmdComplete()){
                    robot.driveTrain.cmdTurn(16,0.25);
                    currentStage = stage._99_BackUpFANCY;
                }

                break;
            case _99_BackUpFANCY:
                if (robot.driveTrain.getCmdComplete())     {
                    robot.driveTrain.CmdDrive(20,-168,0.35,2);
                    robot.launcher.cmdOutfar();
                    //runtime.reset();
                    currentStage = stage._100_Runtimefix;
                }

                break;
            case _100_Runtimefix:
                if (robot.driveTrain.getCmdComplete())     {
                    runtime.reset(); //was negative
                    currentStage = stage._101_AutoAdjust3;
                }

                break;

            case _101_AutoAdjust3:
                if (runtime.milliseconds() >=500) {
                    robot.driveTrain.cmdTurn((int)Math.round(robot.targetAngleCalc() -2), 0.20);
                    runtime.reset();
                    currentStage = stage._102_Launch3;
                }


                break;
            case _102_Launch3:
                //if (runtime.milliseconds >= 1000)       {
                if (robot.driveTrain.getCmdComplete())     {
                    robot.launcherBlocker.cmdUnBlock();
                    robot.transitionRoller.cmdSpin();
                    robot.intake.cmdFoward();
                    runtime.reset();
                    currentStage = stage._103_stopLaunch3;
                }

                break;
            case _103_stopLaunch3:
                if (runtime.milliseconds() >= 1500)     {
                    robot.driveTrain.CmdDrive(3,-168,.25,0);
                    robot.driveTrain.cmdTurn(-58,0.35);
                    robot.launcher.cmdStop();
                    robot.launcherBlocker.cmdBlock();
                    currentStage = stage._105_MoveForward3;
                }


                break;
            case _105_MoveForward3:
                if (robot.driveTrain.getCmdComplete())     {
                    robot.driveTrain.CmdDrive(50,-60,0.40,-90); // possibly SLAM into wall with MORE speed!
                    robot.transitionRoller.cmdSpin();
                    robot.intake.cmdFoward();
                    currentStage = stage._106_LastTurn;
                }

                break;
            case _106_LastTurn:
                if (robot.driveTrain.getCmdComplete())     {
                    robot.driveTrain.CmdDrive(25,-110,0.40,-65);
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
        _30_MoveForward,
        _40_TurnLeft1,
        _50_MoveForward2,
        _53_TurnToGate,
        _57_UnturnToGate,
        _60_MoveBack,
        _65_UnturnCause,
        _66_Driveback,
        _68_Alighn,
        _69_AutoAdjust2,
        _70_Launch2,
        _75_MoveForward2,
        _80_TurnToArtifact2,
        _90_Forward3,
        _95_turn2,
        _97_backup2,
        _99_BackUpFANCY,
        _100_Runtimefix,
        _101_AutoAdjust3,
        _102_Launch3,
        _103_stopLaunch3,
        _105_MoveForward3,
        _106_LastTurn,
        _107_ResetGyro,
        _110_End


    }
}

