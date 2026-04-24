package org.firstinspires.ftc.teamcode.Autons;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Common.Settings;
import org.firstinspires.ftc.teamcode.Hardware.Robot;


@Disabled
@Autonomous(name = "RedNearThreeCycle", group = "Auton")
// @Autonomous(...) is the other common choice

public class RedNearThreeCycle extends OpMode {

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
                currentStage = stage._20_DriveBack;
                break;


            case _20_DriveBack:
                if (robot.driveTrain.getCmdComplete())     {
                    robot.driveTrain.CmdDrive(25,180,0.35,0);
                    robot.launcher.cmdOuttouch();
                    currentStage = stage._25_Turn;
                }
                break;

            case _25_Turn:
                if (robot.driveTrain.getCmdComplete())     {
                    robot.driveTrain.cmdTurn(-5,0.25);
                    runtime.reset();
                    currentStage = stage._30_Shoot1;
                }

                break;
            case _30_Shoot1:
                if (runtime.milliseconds() >=1500)  {
                    robot.intake.cmdFoward();
                    robot.transitionRoller.cmdSpin();
                    robot.launcherBlocker.cmdUnBlock();
                    runtime.reset();
                    currentStage = stage._40_LauncherStop;
                }
                break;
            case _40_LauncherStop:
                if (runtime.milliseconds() >=1500){
                    robot.driveTrain.cmdTurn(0,0.25);
                    robot.launcherBlocker.cmdBlock();
                    robot.transitionRoller.cmdStop();
                    //robot.launcher.cmdStop();
                    runtime.reset();
                    currentStage = stage._45_Forward2;
                }
                break;
            case _45_Forward2:
                if (robot.driveTrain.getCmdComplete())     {
                    robot.driveTrain.CmdDrive(12,0,0.35,0);
                    currentStage = stage._50_Left1;
                }
                break;

            case _50_Left1:
                if (robot.driveTrain.getCmdComplete())     {
                    robot.driveTrain.CmdDrive(53,90,0.35,0);
                    currentStage = stage._60_Foward1;
                }
                break;

            case _60_Foward1:
                if (robot.driveTrain.getCmdComplete())    {
                    robot.transitionRoller.cmdSpin();
                    robot.intake.cmdFoward();
                    robot.driveTrain.CmdDrive(23,0,0.20,0);
                    currentStage = stage._70_Backwards1;
                }

                break;

            case _70_Backwards1:
                if (robot.driveTrain.getCmdComplete())    {
                    robot.driveTrain.CmdDrive(27,180,0.35,0);
                    currentStage = stage._80_TurnToLaunch1;
                }
                break;

            case _80_TurnToLaunch1:
                if (robot.driveTrain.getCmdComplete())    {
                    robot.driveTrain.cmdTurn(-45,0.30);
                    robot.intake.cmdStop();
                    robot.transitionRoller.cmdStop();
                    robot.launcher.cmdOutnear();
                    runtime.reset();
                    currentStage = stage._90_Shoot2;
                }

                break;
            case _90_Shoot2:
                if (runtime.milliseconds() >=1500)  {
                    robot.launcherBlocker.cmdUnBlock();
                    robot.transitionRoller.cmdSpin();
                    robot.intake.cmdFoward();
                    runtime.reset();
                    currentStage = stage._100_Stop;


                }
                break;
            case _100_Stop:
                if (runtime.milliseconds() >=1500) {
                    robot.launcher.cmdStop();
                    robot.transitionRoller.cmdStop();
                    robot.launcherBlocker.cmdBlock();
                    robot.intake.cmdStop();
                    robot.driveTrain.cmdTurn(0,0.30);
                    runtime.reset();
                    currentStage = stage._110_Left2;
                }

                /*
                break;
            case _105_TurnForward:
                if (robot.driveTrain.getCmdComplete())     {
                    robot.driveTrain.CmdDrive(20,90,0.35,0);
                    currentStage = stage._110_Left2;
                }
                 */

                break;
            case _110_Left2:
                if (robot.driveTrain.getCmdComplete())     {
                    robot.driveTrain.CmdDrive(24,90,0.35,0);
                    robot.transitionRoller.cmdSpin();
                    robot.intake.cmdFoward();
                    currentStage = stage._120_Forward3;
                }

                break;
            case _120_Forward3:
                if (robot.driveTrain.getCmdComplete())    {
                    robot.driveTrain.CmdDrive(33,0,0.35,0);
                    currentStage = stage._130_Backwards2;
                }

                break;
            case _130_Backwards2:
                if (robot.driveTrain.getCmdComplete())    {
                    robot.driveTrain.CmdDrive(34,180,0.35,0);
                    currentStage = stage._140_Right1;
                }

                break;
            case _140_Right1:
                if (robot.driveTrain.getCmdComplete())    {
                    robot.driveTrain.CmdDrive(28,-90,0.35,0);
                    robot.intake.cmdStop();
                    robot.transitionRoller.cmdStop();
                    robot.launcher.cmdOuttouch();
                    currentStage = stage._150_TurnToLaunch2;
                }

                break;
            case _150_TurnToLaunch2:
                if (robot.driveTrain.getCmdComplete())    {
                    robot.driveTrain.cmdTurn(-45,0.30);
                    runtime.reset();
                    currentStage = stage._160_Shoot3;
                }

                break;
            case _160_Shoot3:
                if (runtime.milliseconds() >=1000)    {
                    robot.launcherBlocker.cmdUnBlock();
                    robot.transitionRoller.cmdSpin();
                    robot.intake.cmdFoward();
                    runtime.reset();
                    currentStage = stage._170_Stop;
                }

                break;
            case _170_Stop:
                if (runtime.milliseconds() >=1500)    {
                    //robot.driveTrain.CmdDrive(11,180,0.35,0);
                    robot.intake.cmdStop();
                    robot.transitionRoller.cmdStop();
                    robot.launcher.cmdStop();
                    robot.launcherBlocker.cmdBlock();
                    robot.driveTrain.cmdTurn(0,0.30);
                    runtime.reset();
                    currentStage = stage._180_Right2;
                }

                break;
            case _180_Right2:
                if (runtime.milliseconds() >=100)  {
                   robot.driveTrain.CmdDrive(20,60,0.35,0);
                    currentStage = stage._200_End;
                }
/*
                break;
            case _190_Backwards3:
                if (robot.driveTrain.getCmdComplete())  {
                    robot.driveTrain.CmdDrive(12,180,0.35,-45);
                    currentStage = stage._200_End;
                }

 */

                break;
            case _200_End:
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
        _10_turn,
        _20_DriveBack,
        _25_Turn,
        _30_Shoot1,
        _40_LauncherStop,
        _45_Forward2,
        _50_Left1,
        _60_Foward1,
        _70_Backwards1,
        _80_TurnToLaunch1,
        _90_Shoot2,
        _100_Stop,
        _105_TurnForward,
        _110_Left2,
        _120_Forward3,
        _130_Backwards2,
        _140_Right1,
        _150_TurnToLaunch2,
        _160_Shoot3,
        _170_Stop,
        _180_Right2,
        _190_Backwards3,
        _200_End



    }
}

