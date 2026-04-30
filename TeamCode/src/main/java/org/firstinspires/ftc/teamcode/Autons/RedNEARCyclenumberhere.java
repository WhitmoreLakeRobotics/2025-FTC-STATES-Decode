package org.firstinspires.ftc.teamcode.Autons;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Common.Settings;
import org.firstinspires.ftc.teamcode.Hardware.Robot;

@Disabled
@Autonomous(name = "RedNEARCyclenumberhere", group = "Auton")
// @Autonomous(...) is the other common choice

public class RedNEARCyclenumberhere extends OpMode {

    //RobotComp robot = new RobotComp();
    Robot robot = new Robot();

    private int Xpos = 0;
    //private int Ypos = 0;

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
                currentStage = stage._10_DriveToPreLaunch;
                break;
            case _10_DriveToPreLaunch:
                StartLauncher(true);
                robot.driveTrain.CmdDrive(12,180,0.35,0);
                currentStage = stage._15_AutoTurning1;

                break;
            case _15_AutoTurning1:
                autoTurn(stage._20_Launch1);
                break;

            case _20_Launch1:
                if(robot.driveTrain.getCmdComplete()){
                    dolaunch_process();
                    //robot.driveTrain.CmdDrive(0,0,0.0,0);
                    currentStage = stage._30_DriveToPickup1;
                }
                break;
            case _30_DriveToPickup1:
                if (runtime.milliseconds() >= 1000 || robot.sensors.Empty)     { //change or update maybe
                    endlaunch_process();
                    robot.driveTrain.CmdDrive(48,90,0.35,0);
                    currentStage = stage._40_PickupSpike1;
                }
                break;
/*
            case _35_Turning1:
                if (robot.driveTrain.getCmdComplete())     { //change or update maybe
                    //endlaunch_process();
                    robot.intake.cmdFoward();
                    robot.driveTrain.cmdTurn(90,0.35);
                    currentStage = stage._40_PickupSpike1;
                }
                break;
 */

            case _40_PickupSpike1:
                PickupSetup(stage._50_InterruptedBackup1,12); //change distance
                break;

            case _50_InterruptedBackup1:
                if (robot.driveTrain.getCmdComplete()){
                   InterruptedPickup(stage._60_BackToLaunch1);
                    //currentStage = stage._60_BackToLaunch1;
                }

            case _55_Turning15:
                if (robot.driveTrain.getCmdComplete()){
                    robot.driveTrain.cmdTurn(-45,0.35);
                    currentStage = stage._65_AutoTurning2;
                }

                break;
            case _60_BackToLaunch1:
                if (robot.driveTrain.getCmdComplete()) {
                    robot.driveTrain.CmdDrive(24,-180,0.35,0);
                    StartLauncher(false);
                    currentStage = stage._55_Turning15;
                }

                break;
            case _65_AutoTurning2:
                if (robot.driveTrain.getCmdComplete()){
                    autoTurn(stage._70_Launch2);
                    //currentStage = stage._60_BackToLaunch1;
                }

                break;
            case _70_Launch2:
                if (robot.driveTrain.getCmdComplete()){
                    dolaunch_process();
                    currentStage = stage._80_DriveToPickup2;
                }

                break;
            case _80_DriveToPickup2:
                if (robot.driveTrain.getCmdComplete()){
                    endlaunch_process();
                    robot.intake.cmdFoward();
                    robot.driveTrain.cmdTurn(0,0.35);

                    currentStage = stage._90_PickupCorner1;
                }
                break;
            case _90_PickupCorner1:
                    PickupSetup(stage._100_InterruptedBackup2,24);  // fix
                break;
            case _100_InterruptedBackup2:
                InterruptedPickup(stage._110_BackToLaunch2);
                break;
            case _110_BackToLaunch2:
                if(robot.driveTrain.getCmdComplete()){
                    robot.driveTrain.cmdTurn(-45,0.35);
                    StartLauncher(false);
                    currentStage = stage._115_AutoTurning3;
                }

                    break;
            case _115_AutoTurning3:
                autoTurn(stage._120_Launch3);
                break;

            case _120_Launch3:
                if(robot.driveTrain.getCmdComplete()) {
                    dolaunch_process();
                    currentStage = stage._135_Turning2;
                }
                break;

            case _130_DriveToPickup3:
                if(runtime.milliseconds() >= 1000 || robot.sensors.Empty) {
                   endlaunch_process();
                   robot.driveTrain.CmdDrive(24,0,0.35,0);
                    currentStage = stage._140_PickupSpike2;
                }
                break;

            case _135_Turning2:
                if(robot.driveTrain.getCmdComplete()) {
                    robot.driveTrain.cmdTurn(0,0.35);
                    currentStage = stage._130_DriveToPickup3;
                }
                break;

            case _140_PickupSpike2:
                PickupSetup(stage._150_InterruptedBackup3,12);
                    //currentStage = stage._150_InterruptedBackup3;
                break;

            case _150_InterruptedBackup3:
                InterruptedPickup(stage._160_BackToLaunch3);
                //currentStage = stage._150_InterruptedBackup3;
                break;

            case _155_Turning25:
                if(robot.driveTrain.getCmdComplete()){
                    robot.driveTrain.cmdTurn(-45,0.35);
                    currentStage = stage._165_AutoTurning4;
                }
                break;

            case _160_BackToLaunch3:
                if(robot.driveTrain.getCmdComplete()){
                    StartLauncher(false);
                    robot.driveTrain.CmdDrive(24, 180 ,0.35 ,0);
                    currentStage = stage._155_Turning25;
                }
                break;

            case _165_AutoTurning4:
                if(robot.driveTrain.getCmdComplete()){
                    autoTurn(stage._170_Launch4);
                }
                break;

            case _170_Launch4:
                if(robot.driveTrain.getCmdComplete()){
                    dolaunch_process();
                    currentStage = stage._180_DriveToPickup4;
                }
                break;

            case _180_DriveToPickup4:
                if(runtime.milliseconds() >= 1000 || robot.sensors.Empty){
                    endlaunch_process();
                    robot.driveTrain.cmdTurn(0,0.35);
                    currentStage = stage._190_PickupCorner2;
                }
                break;

            case _190_PickupCorner2:
                PickupSetup(stage._200_InterruptedBackup4,24); //fix
                break;

            case _200_InterruptedBackup4:
                InterruptedPickup(stage._210_BackToLaunch4);
                break;

            case _210_BackToLaunch4:
                StartLauncher(false);
                robot.driveTrain.cmdTurn(-45,0.35);
                currentStage = stage._220_AutoTurning5;
                break;

            case _220_AutoTurning5:
                autoTurn(stage._230_Launch5);
                break;

            case _230_Launch5:
                if(robot.driveTrain.getCmdComplete()) {
                    dolaunch_process();
                    currentStage = stage._240_SlowTurnToDeath;
                }
                break;

            case _240_SlowTurnToDeath:
                if(runtime.milliseconds() >= 1000 || robot.sensors.Empty) {
                    endlaunch_process();
                   robot.driveTrain.cmdTurn(0, 0.35);
                    currentStage = stage._250_QuickScaryDeath;
                }
                break;

            case _250_QuickScaryDeath:
                if(robot.driveTrain.getCmdComplete()) {
                    robot.driveTrain.CmdDrive(24,90,0.50,0);
                    currentStage = stage._260_End;
                }
                break;

            case _260_End:
                if(robot.driveTrain.getCmdComplete()){
                    robot.stop();
                }

                break;
        }



    }  //  loop

    private void dolaunch_process() {
        robot.launcherBlocker.cmdUnBlock();
        robot.launcher.launching = true;
        robot.transitionRoller.cmdSpin();
        robot.intake.cmdFoward();
        runtime.reset();
    }

    private void endlaunch_process() {
        robot.launcherBlocker.cmdBlock();
        robot.launcher.launching = false;
        robot.autoRPM.Measure = false;
        robot.launcher.cmdStop();
    }

    public void StartLauncher(boolean Touch){
        if(robot.limey.getTagID() > 1){
            robot.autoRPM.Measure = true;
        }else{
            if(Touch){
                robot.launcher.cmdOuttouch();
            }else {
                robot.launcher.cmdOutnear();
            }
        }
    }

    private void autoTurn(stage NextStage){
        if(robot.driveTrain.getCmdComplete()) {
            robot.driveTrain.cmdTurn((int) Math.round(robot.targetAngleCalc()), 0.35);
            runtime.reset();
            if(runtime.milliseconds() >= 500) {
                currentStage = NextStage;
            }
        }
    }

    private void PickupSetup(stage NextStage2, int DistanceOfPickup){
        if (robot.driveTrain.getCmdComplete())  {
            if(Xpos < DistanceOfPickup && !robot.intake.autoStopped) {
                robot.driveTrain.CmdDrive(1, 90, 0.35, 90);
                Xpos = Xpos + 1;
            }else if(robot.intake.autoStopped && Xpos < DistanceOfPickup){
                currentStage = NextStage2;
            }else{
                currentStage = NextStage2;
            }
        }
    }

    private void InterruptedPickup(stage NextStage3){
        if(robot.driveTrain.getCmdComplete()){
            robot.driveTrain.CmdDrive(Xpos,-90,0.35,90);
            Xpos = 0;
            currentStage = NextStage3;
        }
    }

      //Code to run ONCE after the driver hits STOP

    @Override
    public void stop() {
        robot.stop();
    }

    private enum stage {
        _unknown,
        _00_preStart,
        _10_DriveToPreLaunch,
        _15_AutoTurning1,
        _20_Launch1,
        _30_DriveToPickup1,
        _35_Turning1,
        _40_PickupSpike1,
        _50_InterruptedBackup1,
        _55_Turning15,
        _60_BackToLaunch1,
        _65_AutoTurning2,
        _70_Launch2,
        _80_DriveToPickup2,
        _90_PickupCorner1,
        _100_InterruptedBackup2,
        _110_BackToLaunch2,
        _115_AutoTurning3,
        _120_Launch3,
        _130_DriveToPickup3,
        _135_Turning2,
        _140_PickupSpike2,
        _150_InterruptedBackup3,
        _155_Turning25,
        _160_BackToLaunch3,
        _165_AutoTurning4,
        _170_Launch4,
        _180_DriveToPickup4,
        _190_PickupCorner2,
        _200_InterruptedBackup4,
        _210_BackToLaunch4,
        _220_AutoTurning5,
        _230_Launch5,
        _240_SlowTurnToDeath,
        _250_QuickScaryDeath,
        _260_End


    }
    //Farb = Ferb + Far
    //Phinearus = Phineas + Near
}

