package org.firstinspires.ftc.teamcode.Hardware;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Common.CommonLogic;


/**
 * Base class for FTC Team 8492 defined hardware
 */

public class Sensors extends BaseHardware {


    private ElapsedTime runtime = new ElapsedTime();
    /**
     * The {@link #telemetry} field contains an object in which a user may accumulate data which
     * is to be transmitted to the driver station. This data is automatically transmitted to the
     * driver station on a regular, periodic basis.
     */
    public Telemetry telemetry = null;
    //private ColorRangeSensor IntakeSensor;
    //private DistanceSensor RearLeftSensor






    private Servo PeaLight;
    private Servo PeaDark;
    public ColorRangeSensor NTKAP1; // expansion hub port 2
    public ColorRangeSensor NTKAP2;
    public ColorRangeSensor NTKAP3;

    public Distance1 CurrentDistance1;
    public Distance2 CurrentDistance2;
    public Distance3 CurrentDistance3;
    public Color CurrentColor;


    public static final double Green = 0.5;
    public static final double Red = 0.28;
    public static final double Yellow = 0.388;
    public static final double Purple = 0.722;
    public static final double Blue = 0.6111;
    public static final double Orange = 0.333;
    public static final double Off = 0;

    private double NTKAP1distance;
    private double NTKAP2distance;
    private double NTKAP3distance;

    public boolean AtIntakeStop = true;
    public boolean initLight1 = false;
    public boolean initLight2 = false;
    public boolean Empty = false;
    public ElapsedTime GameTime = new ElapsedTime();
    public int Total = 120;
    public int TimeRemaining = (int)(Total - GameTime.seconds());

    public ElapsedTime initLightTime = new ElapsedTime();
    public ElapsedTime stable = new ElapsedTime();









    private boolean cmdComplete = true;
    private Mode CurrentMode = Mode.STOP;


    private int SensorBlue;
    private int SensorRed;
    private int SensorGreen;


    /**
     * Hardware Mappings
     */
    public HardwareMap hardwareMap = null; // will be set in Child class


    /**
     * BaseHardware constructor
     * <p>
     * The op mode name should be unique. It will be the name displayed on the driver station. If
     * multiple op modes have the same name, only one will be available.
     */
   /*public Swing_Arm_And_Lift() {


   }*/


    /**
     * User defined init method
     * <p>
     * This method will be called once when the INIT button is pressed.
     */
    public void init(){
        //DeliverySensor = hardwareMap.get(ColorSensor.class, "DeliveryS");


        NTKAP3 = hardwareMap.get(ColorRangeSensor.class, "NTKAP3");
        NTKAP2 = hardwareMap.get(ColorRangeSensor.class, "NTKAP2");
        NTKAP1 = hardwareMap.get(ColorRangeSensor.class, "NTKAP1");
        PeaLight = hardwareMap.get(Servo.class,"PeaLight");
        PeaDark = hardwareMap.get(Servo.class,"PeaDark");

        initLightTime.reset();
        initLight1 = true;
    }


    /**
     * User defined init_loop method
     * <p>
     * This method will be called repeatedly when the INIT button is pressed.
     * This method is optional. By default this method takes no action.
     */
    public void init_loop() {

        if(initLight1 && initLightTime.milliseconds() >= 2000){
            cmdORANGE();
            PeaDark.setPosition(Off);
            initLight1 = false;
            initLightTime.reset();
            initLight2 = true;
        }

        if(initLight2 && initLightTime.milliseconds() >= 2000){
            cmdOFF();
            PeaDark.setPosition(Orange);
            initLight2 = false;
            initLightTime.reset();
            initLight1 = true;
        }

        /**
         * User defined init_loop method
         * <p>
         * This method will be called repeatedly when the INIT button is pressed.
         * This method is optional. By default this method takes no action.
         */


//         telemetry.addData("FLDS1 Pos " , FLDS1.getDistance(DistanceUnit.INCH)) ;
    }


    /**
     * User defined start method.
     * <p>
     * This method will be called once when the PLAY button is first pressed.
     * This method is optional. By default this method takes not action.
     * Example usage: Starting another thread.
     */
    public void start(){
        initLight1 = false;
        initLight2 = false;
        cmdRED();
        PeaDark.setPosition(Off);
        GameTime.reset();
    }


    /**
     * User defined loop method
     * <p>
     * This method will be called repeatedly in a loop while this op mode is running
     */
    public void loop(){
        TimeRemaining = (int)(Total - GameTime.seconds());

        if (NTKAP1distance <= 8) {
            CurrentDistance1 = Distance1.FILLED1;
            Empty = false;
        } else {
            CurrentDistance1 = Distance1.MISSING1;
            stable.reset();
        }

        if (NTKAP2distance <= 8) {
            CurrentDistance2 = Distance2.FILLED2;
            Empty = false;
        } else {
            CurrentDistance2 = Distance2.MISSING2;
            stable.reset();
        }

        if (NTKAP3distance <= 8) {
            CurrentDistance3 = Distance3.FILLED3;
            Empty = false;
        } else {
            CurrentDistance3 = Distance3.MISSING3;
            stable.reset();
        }

        if(CurrentDistance1 == Distance1.MISSING1 && CurrentDistance2 == Distance2.MISSING2 && CurrentDistance3 == Distance3.MISSING3){
            if(stable.milliseconds() >= 600){
                Empty = true;
            }


        }

        getDistNTKAP1();
        getDistNTKAP2();
        getDistNTKAP3();

        switch (TimeRemaining){

            case 35:
                PeaDark.setPosition(Yellow);
                break;

            case 20:
                PeaDark.setPosition(Purple);
                break;

            case 10:
                PeaDark.setPosition(Red);
                break;

            case 7:
                PeaDark.setPosition(Green);
                break;

            default:
        }
    }

    public void doStop(){
        CurrentMode = Mode.STOP;
        cmdComplete = true;
    }

    /**
     * User defined stop method
     * <p>
     * This method will be called when this op mode is first disabled
     * <p>
     * The stop method is optional. By default this method takes no action.
     */

    public void stop(){


    }

/*
   public TargetType getSlotArtifact(ColorSensor v3) {
        int red1 = v3.red();
        int green1 = v3.green();
        int blue1 = v3.blue();


        if ((CommonLogic.inRange(red1,TargetType.GREENT.red,TargetType.GREENT.redTol ))
                &&(CommonLogic.inRange(blue1,TargetType.GREENT.blue,TargetType.GREENT.blueTol ))
                &&(CommonLogic.inRange(green1,TargetType.GREENT.green,TargetType.GREENT.greenTol))
        ){
            return TargetType.GREENT;
        }else if ((CommonLogic.inRange(red1,TargetType.PURPLET.red,TargetType.PURPLET.redTol ))
                &&(CommonLogic.inRange(blue1,TargetType.PURPLET.blue,TargetType.PURPLET.blueTol ))
                &&(CommonLogic.inRange(green1,TargetType.PURPLET.green,TargetType.PURPLET.greenTol))
        ){
            return TargetType.PURPLET;
        }else {
            return TargetType.UNKNOWNT;
        }

    }

*/

    public enum Mode{
        STOP
    }


    public void cmdRED(){
        PeaLight.setPosition(Red);
        CurrentColor = Color.RED;
        //timerun.reset();
    }


    public void cmdGREEN(){
        PeaLight.setPosition(Green);
        CurrentColor = Color.GREEN;
        //timerun.reset();
    }


    public void cmdYELLOW(){
        PeaLight.setPosition(Yellow);
        CurrentColor = Color.YELLOW;
        //timerun.reset();
    }


    public void cmdPURPLE(){
        PeaLight.setPosition(Purple);
        CurrentColor = Color.PURPLE;
        //timerun.reset();
    }


    public void cmdBLUE(){
        PeaLight.setPosition(Blue);
        CurrentColor = Color.BLUE;
    }


    public void cmdORANGE(){
        PeaLight.setPosition(Orange);
        CurrentColor = Color.ORANGE;
        //timerun.reset();
    }


    public void cmdOFF(){
        PeaLight.setPosition(Off);
        CurrentColor = Color.OFF;
        //timerun.reset();
    }




    public enum TargetType {
        GREENT(25, 5, 75,25,1,3 ),
        PURPLET(6, 1, 1, 2, 7,1 ),
        UNKNOWNT(1, 1, 1,1,1,1);


        private int red;
        private int  redTol;
        private int blue;
        private int blueTol;
        private int green;
        private int greenTol;




        TargetType(int red,int redTol,int blue,int blueTol,int green,int greenTol) {
            this.red = red;
            this.redTol = redTol;
            this.blue = blue;
            this.blueTol = blueTol;
            this.green = green;
            this.greenTol = greenTol;
        }

    }

    private void getDistNTKAP2() {
        NTKAP2distance = NTKAP2.getDistance(DistanceUnit.CM);
    }
    private void getDistNTKAP3() {
        NTKAP3distance = NTKAP3.getDistance(DistanceUnit.CM);
    }
    private void getDistNTKAP1() {
        NTKAP1distance = NTKAP1.getDistance(DistanceUnit.CM);
    }

    public enum Distance3 {
        FILLED3,
        MISSING3
    }

    public enum Distance2 {
        FILLED2,
        MISSING2
    }

    public enum Distance1 {
        FILLED1,
        MISSING1
    }

    public enum Color {
        GREEN,
        RED,
        YELLOW,
        PURPLE,
        BLUE,
        ORANGE,
        OFF
    }

}