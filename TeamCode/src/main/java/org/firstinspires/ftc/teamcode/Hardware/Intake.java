package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Common.CommonLogic;

/**
 * Base class for FTC Team 8492 defined hardware
 */
public class Intake extends BaseHardware{

    public Telemetry telemetry = null;

    public HardwareMap hardwareMap = null; // will be set in Child class
    private TransitionRoller transitionRoller = new TransitionRoller();

    private DcMotorEx NTKM01;
    public Mode CurrentMode;
    private double NTKM01Power;

    public final double minPower = -1.0;
    public final double maxPower = 1.0;

    public static final double stopSpeed = 0;
    public static final double inSpeed = -1;  // use to be 0.5,then 0.75;
    public static final double outSpeed = 0.65;

    public boolean DriverHappy = false;
    public boolean AtIntakeStop = true;
    public boolean InPain = false;
    public boolean MentallyStable = false;
    public boolean autoStopped = false;

    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime timerun = new ElapsedTime();

    public void init() {
        NTKM01 = hardwareMap.get(DcMotorEx.class, "NTKM01");
    }

    public void init_loop(){
    }

    public void start(){
        runtime.reset();
    }

    public void loop(){

        /*
        if((CommonLogic.inRange(getMotorRPM(NTKM01), 600, 600))){
            InPain = true;
        }else{
            InPain = false;
        }
*/

        if(runtime.milliseconds() >= 1000){
            MentallyStable = true;
        }else{
            MentallyStable = false;
        }

    }

    void stop(){
    }

    public void cmdBackward(){
        CurrentMode = Mode.NTKbackward;
        NTKM01.setPower (outSpeed);
    }
    public void cmdFoward(){
        CurrentMode = Mode.NTKforward;
        NTKM01.setPower (inSpeed);
        runtime.reset();
        autoStopped = false;
    }

    public void cmdStop(){
        CurrentMode = Mode.NTKstop;
        NTKM01.setPower (stopSpeed);
        runtime.reset();
    }

    public enum Mode {
        NTKstop,
        NTKforward,
        NTKbackward
    }

    public double getMotorRPM(DcMotorEx motor){
        double ticksPerRevolution = 28; //update and double check
        double gearRatio = 1.0; //update and double check
        double ticksPerSecond = motor.getVelocity();
        return (ticksPerSecond / ticksPerRevolution) * 60 * gearRatio;
    }

}