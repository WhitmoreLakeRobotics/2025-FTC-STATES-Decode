package org.firstinspires.ftc.teamcode.Autons;


import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.Hardware.Robot;


import java.util.Objects;




@Autonomous(name = "GameBoy", group = "PP")
public class GameBoy extends OpMode {
    Robot robot = new Robot();
    private String thisUpdate = "0";
    private TelemetryManager telemetryMU;
    private line currentLine = line.D;
    private smile currentSmile = smile.F1;
    private GalagaL currentGalagaL = GalagaL.Sa1;
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime runtimeGa = new ElapsedTime();
    private ElapsedTime runtimeGb = new ElapsedTime();
    private ElapsedTime gametime = new ElapsedTime();


    //None
    public String A = "* * * * * * *";
    public String B = "* * * * * * *";
    public String C = "* * * * * * *";
    public String D = "* * * A * * *";
    public String E = "* * * * * * *";
    public String F = "* * * * * * *";
    public String G = "* * * * * * *";


    //Galaga
    public String A2 = "* * * * * * *";
    public String B2 = "* * * * * * *";
    public String C2 = "* * * * * * *";
    public String D2 = "* * * * * * *";
    public String E2 = "* * * * * * *";
    public String F2 = "* * * * * * *";
    public String G2 = "* * * A * * *";
    public int PointsG = 10;
    public int HighScoreG = 0;
    public boolean APressedG = false;
    public boolean EndGalaga = false;


    //Intro
    public String I1 = "*  *  *  *  *";
    public String I2 = "*  0  *  0  *";
    public String I3 = "*  *  *  *  *";
    public String I4 = "*  =  =  =  *";
    public String I5 = "*  *  *  *  *";
    public String I6 = "*  *  *  *  *";


    //Game Chooser
    public String Game = "None"; //"Galaga"




    @Override
    public void init() {
        robot.hardwareMap = hardwareMap;
        robot.telemetry = telemetry;
        robot.init();
        runtime.reset();
        EndGalaga = false;
    }


    @Override
    public void init_loop() {


        telemetry.addData("I1",I1);
        telemetry.addData("I2",I2);
        telemetry.addData("I3",I3);
        telemetry.addData("I4",I4);
        telemetry.addData("I5",I5);
        telemetry.addData("I6",I6);


        switch (currentSmile){
            case F1:
                if(runtime.milliseconds() >= 750) {
                    I1 = "*  *  *  *  *";
                    I2 = "*  0  *  0  *";
                    I3 = "*  *  *  *  *";
                    I4 = "*  =  =  =  *";
                    I5 = "*  *  *  *  *";
                    I6 = "*  *  *  *  *";
                    runtime.reset();
                    currentSmile = smile.F2;
                }
                break;
            case F2:
                if(runtime.milliseconds() >= 750) {
                    I1 = "*  *  *  *  *";
                    I2 = "*  0  *  0  *";
                    I3 = "*  *  *  *  *";
                    I4 = "*  o  o  o  *";
                    I5 = "*  *  *  *  *";
                    I6 = "W E L C O M E";
                    runtime.reset();
                    currentSmile = smile.F3;
                }
                break;
            case F3:
                if(runtime.milliseconds() >= 750) {
                    I1 = "*  *  *  *  *";
                    I2 = "*  0  *  0  *";
                    I3 = "o  *  *  *  o";
                    I4 = "*  o  o  o  *";
                    I5 = "*  *  *  *  *";
                    I6 = "*  *  *  *  *";
                    runtime.reset();
                    currentSmile = smile.F4;
                }
                break;
            case F4:
                if(runtime.milliseconds() >= 750) {
                    I1 = "*  *  *  *  *";
                    I2 = "*  _  *  _  *";
                    I3 = "o  *  *  *  o";
                    I4 = "*  o  o  o  *";
                    I5 = "*  *  *  *  *";
                    I6 = "W E L C O M E";
                    runtime.reset();
                    currentSmile = smile.F5;
                }
                break;
            case F5:
                if(runtime.milliseconds() >= 500) {
                    I1 = "*  *  *  *  *";
                    I2 = "*  0  *  0  *";
                    I3 = "o  *  *  *  o";
                    I4 = "*  o  o  o  *";
                    I5 = "*  *  *  *  *";
                    I6 = "W E L C O M E";
                    runtime.reset();
                    currentSmile = smile.F6;
                }
                break;
            case F6:
                if(runtime.milliseconds() >= 750) {
                    telemetry.addData("Game:",Game);
                    I1 = "";
                    I2 = "";
                    I3 = "";
                    I4 = "";
                    I5 = "";
                    I6 = "";
                    if(gamepad1.dpad_left){
                        if(Game == "None"){
                            Game = "Galaga";
                        }else{
                            Game = "None";
                        }
                    }
                    if(gamepad1.dpad_right){
                        if(Game == "None"){
                            Game = "Galaga";
                        }else{
                            Game = "None";
                        }
                    }
                }
                break;


        }


        robot.init_loop();


    }


    @Override
    public void start () {
        //super.start();
        robot.start();
        gametime.reset();


    }


    @Override
    public void loop() {
        if(Game == "None") {
            telemetry.addData("A", A);
            telemetry.addData("B", B);
            telemetry.addData("C", C);
            telemetry.addData("D", D);
            telemetry.addData("E", E);
            telemetry.addData("F", F);
            telemetry.addData("G", G);


            switch (currentLine) {
                case A:
                    if (gamepad1.dpad_right) {
                        if (A == "A * * * * * *") {
                            A = "* A * * * * *";


                        } else if (A == "* A * * * * *") {
                            A = "* * A * * * *";


                        } else if (A == "* * A * * * *") {
                            A = "* * * A * * *";


                        } else if (A == "* * * A * * *") {
                            A = "* * * * A * *";


                        } else if (A == "* * * * A * *") {
                            A = "* * * * * A *";


                        } else if (A == "* * * * * A *") {
                            A = "* * * * * * A";


                        } else if (A == "* * * * * * A") {
                            A = "A * * * * * *";
                        } else {


                        }
                    }
                    if (gamepad1.dpad_left) {
                        if (A == "A * * * * * *") {
                            A = "* * * * * * A";


                        } else if (A == "* A * * * * *") {
                            A = "A * * * * * *";


                        } else if (A == "* * A * * * *") {
                            A = "* A * * * * *";


                        } else if (A == "* * * A * * *") {
                            A = "* * A * * * *";


                        } else if (A == "* * * * A * *") {
                            A = "* * * A * * *";


                        } else if (A == "* * * * * A *") {
                            A = "* * * * A * *";


                        } else if (A == "* * * * * * A") {
                            A = "* * * * * A *";
                        } else {


                        }
                    }
                    if (gamepad1.dpad_down) {
                        B = A;
                        A = "* * * * * * *";
                        currentLine = line.B0;
                    }
                    if (gamepad1.dpad_up) {
                        G = A;
                        A = "* * * * * * *";
                        currentLine = line.G0;
                    }
                    break;
                case B:
                    if (gamepad1.dpad_right) {
                        if (B == "A * * * * * *") {
                            B = "* A * * * * *";


                        } else if (B == "* A * * * * *") {
                            B = "* * A * * * *";


                        } else if (B == "* * A * * * *") {
                            B = "* * * A * * *";


                        } else if (B == "* * * A * * *") {
                            B = "* * * * A * *";


                        } else if (B == "* * * * A * *") {
                            B = "* * * * * A *";


                        } else if (B == "* * * * * A *") {
                            B = "* * * * * * A";


                        } else if (B == "* * * * * * A") {
                            B = "A * * * * * *";
                        } else {


                        }
                    }
                    if (gamepad1.dpad_left) {
                        if (B == "A * * * * * *") {
                            B = "* * * * * * A";


                        } else if (B == "* A * * * * *") {
                            B = "A * * * * * *";


                        } else if (B == "* * A * * * *") {
                            B = "* A * * * * *";


                        } else if (B == "* * * A * * *") {
                            B = "* * A * * * *";


                        } else if (B == "* * * * A * *") {
                            B = "* * * A * * *";


                        } else if (B == "* * * * * A *") {
                            B = "* * * * A * *";


                        } else if (B == "* * * * * * A") {
                            B = "* * * * * A *";
                        } else {


                        }
                    }
                    if (gamepad1.dpad_down) {
                        B = B;
                        currentLine = line.C0;
                    }
                    if (gamepad1.dpad_up) {
                        A = B;
                        currentLine = line.A0;
                    }
                    break;
                case C:
                    if (gamepad1.dpad_right) {
                        if (C == "A * * * * * *") {
                            C = "* A * * * * *";


                        } else if (C == "* A * * * * *") {
                            C = "* * A * * * *";


                        } else if (C == "* * A * * * *") {
                            C = "* * * A * * *";


                        } else if (C == "* * * A * * *") {
                            C = "* * * * A * *";


                        } else if (C == "* * * * A * *") {
                            C = "* * * * * A *";


                        } else if (C == "* * * * * A *") {
                            C = "* * * * * * A";


                        } else if (C == "* * * * * * A") {
                            C = "A * * * * * *";
                        } else {


                        }
                    }
                    if (gamepad1.dpad_left) {
                        if (C == "A * * * * * *") {
                            C = "* * * * * * A";


                        } else if (C == "* A * * * * *") {
                            C = "A * * * * * *";


                        } else if (C == "* * A * * * *") {
                            C = "* A * * * * *";


                        } else if (C == "* * * A * * *") {
                            C = "* * A * * * *";


                        } else if (C == "* * * * A * *") {
                            C = "* * * A * * *";


                        } else if (C == "* * * * * A *") {
                            C = "* * * * A * *";


                        } else if (C == "* * * * * * A") {
                            C = "* * * * * A *";
                        } else {


                        }
                    }
                    if (gamepad1.dpad_down) {
                        D = C;
                        currentLine = line.D0;
                    }
                    if (gamepad1.dpad_up) {
                        B = C;
                        currentLine = line.B0;
                    }
                    break;
                case D:
                    if (gamepad1.dpad_right) {
                        if (D == "A * * * * * *") {
                            D = "* A * * * * *";


                        } else if (D == "* A * * * * *") {
                            D = "* * A * * * *";


                        } else if (D == "* * A * * * *") {
                            D = "* * * A * * *";


                        } else if (D == "* * * A * * *") {
                            D = "* * * * A * *";


                        } else if (D == "* * * * A * *") {
                            D = "* * * * * A *";


                        } else if (D == "* * * * * A *") {
                            D = "* * * * * * A";


                        } else if (D == "* * * * * * A") {
                            D = "A * * * * * *";
                        } else {


                        }
                    }
                    if (gamepad1.dpad_left) {
                        if (D == "A * * * * * *") {
                            D = "* * * * * * A";


                        } else if (D == "* A * * * * *") {
                            D = "A * * * * * *";


                        } else if (D == "* * A * * * *") {
                            D = "* A * * * * *";


                        } else if (D == "* * * A * * *") {
                            D = "* * A * * * *";


                        } else if (D == "* * * * A * *") {
                            D = "* * * A * * *";


                        } else if (D == "* * * * * A *") {
                            D = "* * * * A * *";


                        } else if (D == "* * * * * * A") {
                            D = "* * * * * A *";
                        } else {


                        }
                    }
                    if (gamepad1.dpad_down) {
                        E = D;
                        currentLine = line.E0;
                    }
                    if (gamepad1.dpad_up) {
                        C = D;
                        currentLine = line.C0;
                    }
                    break;
                case E:
                    if (gamepad1.dpad_right) {
                        if (E == "A * * * * * *") {
                            E = "* A * * * * *";


                        } else if (E == "* A * * * * *") {
                            E = "* * A * * * *";


                        } else if (E == "* * A * * * *") {
                            E = "* * * A * * *";


                        } else if (E == "* * * A * * *") {
                            E = "* * * * A * *";


                        } else if (E == "* * * * A * *") {
                            E = "* * * * * A *";


                        } else if (E == "* * * * * A *") {
                            E = "* * * * * * A";


                        } else if (E == "* * * * * * A") {
                            E = "A * * * * * *";
                        } else {


                        }
                    }
                    if (gamepad1.dpad_left) {
                        if (E == "A * * * * * *") {
                            E = "* * * * * * A";


                        } else if (E == "* A * * * * *") {
                            E = "A * * * * * *";


                        } else if (E == "* * A * * * *") {
                            E = "* A * * * * *";


                        } else if (E == "* * * A * * *") {
                            E = "* * A * * * *";


                        } else if (E == "* * * * A * *") {
                            E = "* * * A * * *";


                        } else if (E == "* * * * * A *") {
                            E = "* * * * A * *";


                        } else if (E == "* * * * * * A") {
                            E = "* * * * * A *";
                        } else {


                        }
                    }
                    if (gamepad1.dpad_down) {
                        F = E;
                        currentLine = line.F0;
                    }
                    if (gamepad1.dpad_up) {
                        D = E;
                        currentLine = line.D0;
                    }
                    break;
                case F:
                    if (gamepad1.dpad_right) {
                        if (F == "A * * * * * *") {
                            F = "* A * * * * *";


                        } else if (F == "* A * * * * *") {
                            F = "* * A * * * *";


                        } else if (F == "* * A * * * *") {
                            F = "* * * A * * *";


                        } else if (F == "* * * A * * *") {
                            F = "* * * * A * *";


                        } else if (F == "* * * * A * *") {
                            F = "* * * * * A *";


                        } else if (F == "* * * * * A *") {
                            F = "* * * * * * A";


                        } else if (F == "* * * * * * A") {
                            F = "A * * * * * *";
                        } else {


                        }
                    }
                    if (gamepad1.dpad_left) {
                        if (F == "A * * * * * *") {
                            F = "* * * * * * A";


                        } else if (F == "* A * * * * *") {
                            F = "A * * * * * *";


                        } else if (F == "* * A * * * *") {
                            F = "* A * * * * *";


                        } else if (F == "* * * A * * *") {
                            F = "* * A * * * *";


                        } else if (F == "* * * * A * *") {
                            F = "* * * A * * *";


                        } else if (F == "* * * * * A *") {
                            F = "* * * * A * *";


                        } else if (F == "* * * * * * A") {
                            F = "* * * * * A *";
                        } else {


                        }
                    }
                    if (gamepad1.dpad_down) {
                        G = F;
                        currentLine = line.G0;
                    }
                    if (gamepad1.dpad_up) {
                        E = F;
                        currentLine = line.E0;
                    }
                    break;
                case G:
                    if (gamepad1.dpad_right) {
                        if (G == "A * * * * * *") {
                            G = "* A * * * * *";


                        } else if (G == "* A * * * * *") {
                            G = "* * A * * * *";


                        } else if (G == "* * A * * * *") {
                            G = "* * * A * * *";


                        } else if (G == "* * * A * * *") {
                            G = "* * * * A * *";


                        } else if (G == "* * * * A * *") {
                            G = "* * * * * A *";


                        } else if (G == "* * * * * A *") {
                            G = "* * * * * * A";


                        } else if (G == "* * * * * * A") {
                            G = "A * * * * * *";
                        } else {


                        }
                    }
                    if (gamepad1.dpad_left) {
                        if (G == "A * * * * * *") {
                            G = "* * * * * * A";


                        } else if (G == "* A * * * * *") {
                            G = "A * * * * * *";


                        } else if (G == "* * A * * * *") {
                            G = "* A * * * * *";


                        } else if (G == "* * * A * * *") {
                            G = "* * A * * * *";


                        } else if (G == "* * * * A * *") {
                            G = "* * * A * * *";


                        } else if (G == "* * * * * A *") {
                            G = "* * * * A * *";


                        } else if (G == "* * * * * * A") {
                            G = "* * * * * A *";
                        } else {


                        }
                    }
                    if (gamepad1.dpad_down) {
                        B = G;
                        currentLine = line.A0;
                    }
                    if (gamepad1.dpad_up) {
                        F = G;
                        currentLine = line.F0;
                    }
                    break;
                case A0:
                    G = "* * * * * * *";
                    B = "* * * * * * *";
                    currentLine = line.A;
                    break;
                case B0:
                    A = "* * * * * * *";
                    C = "* * * * * * *";
                    currentLine = line.B;
                    break;
                case C0:
                    B = "* * * * * * *";
                    D = "* * * * * * *";
                    currentLine = line.C;
                    break;
                case D0:
                    C = "* * * * * * *";
                    E = "* * * * * * *";
                    currentLine = line.D;
                    break;
                case E0:
                    D = "* * * * * * *";
                    F = "* * * * * * *";
                    currentLine = line.E;
                    break;
                case F0:
                    E = "* * * * * * *";
                    G = "* * * * * * *";
                    currentLine = line.F;
                    break;
                case G0:
                    F = "* * * * * * *";
                    A = "* * * * * * *";
                    currentLine = line.G;
                    break;
            }
        }else if(Game == "Galaga"){


            if(!EndGalaga) {
                if (PointsG >= HighScoreG) {
                    HighScoreG = PointsG;
                }


                telemetry.addData("A", A2);
                telemetry.addData("B", B2);
                telemetry.addData("C", C2);
                telemetry.addData("D", D2);
                telemetry.addData("E", E2);
                telemetry.addData("F", F2);
                telemetry.addData("G", G2);
                telemetry.addData("Points:", PointsG);
                telemetry.addData("HighScore:", HighScoreG);


                if (PointsG <= 0) {
                    runtimeGa.reset();
                    EndGalaga = true;
                }


                if (gamepad1.a) {
                    runtimeGa.reset();
                    F2 = G2;
                    APressedG = true;
                }
                if (APressedG) {
                    if (runtimeGa.milliseconds() >= 250 && !Objects.equals(F2, "* * * * * * *")) {
                        runtimeGa.reset();
                        E2 = F2;
                    } else if (runtimeGa.milliseconds() >= 250 && !Objects.equals(E2, "* * * * * * *")) {
                        runtimeGa.reset();
                        D2 = E2;
                        F2 = "* * * * * * *";
                    } else if (runtimeGa.milliseconds() >= 250 && !Objects.equals(D2, "* * * * * * *")) {
                        runtimeGa.reset();
                        C2 = D2;
                        E2 = "* * * * * * *";
                    } else if (runtimeGa.milliseconds() >= 250 && !Objects.equals(C2, "* * * * * * *")) {
                        runtimeGa.reset();
                        B2 = C2;
                        D2 = "* * * * * * *";
                    } else if (runtimeGa.milliseconds() >= 250 && !Objects.equals(B2, "* * * * * * *")) {
                        C2 = "* * * * * * *";
                        if (A2 == B2) {
                            PointsG = PointsG + 3;
                        }else{
                            PointsG = PointsG - 1;
                        }
                        B2 = "* * * * * * *";
                        runtimeGb.reset();
                        A2 = "* * * * * * *";
                        currentGalagaL = GalagaL.D2;
                        APressedG = false;
                    }
                }
                if (gamepad1.dpad_right) {
                    if (G2 == "* * * A * * *") {
                        G2 = "* * * * A * *";
                    } else if (G2 == "* * * * A * *") {
                        G2 = "* * * * * A *";
                    } else if (G2 == "* * * * * A *") {
                        G2 = "* * * * * * A";
                    } else if (G2 == "* * * * * * A") {
                    } else if (G2 == "A * * * * * *") {
                        G2 = "* A * * * * *";
                    } else if (G2 == "* * A * * * *") {
                        G2 = "* * * A * * *";
                    } else {
                    }
                }
                if (gamepad1.dpad_left) {
                    if (G2 == "* * * A * * *") {
                        G2 = "* * A * * * *";
                    } else if (G2 == "* * * * A * *") {
                        G2 = "* * * A * * *";
                    } else if (G2 == "* * * * * A *") {
                        G2 = "* * * * A * *";
                    } else if (G2 == "* * * * * * A") {
                        G2 = "* * * * * A *";
                    } else if (G2 == "A * * * * * *") {
                    } else if (G2 == "* * A * * * *") {
                        G2 = "* A * * * * *";
                    } else {
                    }
                }
                switch (currentGalagaL) {
                    case Sa1:
                        A2 = "* * * A * * *";
                        B2 = "* * * * * * *";
                        C2 = "* * * * * * *";
                        D2 = "* * * * * * *";
                        E2 = "* * * * * * *";
                        F2 = "* * * * * * *";
                        G2 = "* * * A * * *";
                        PointsG = 10;
                        APressedG = false;
                        runtimeGb.reset();
                        currentGalagaL = GalagaL.D2;


                        break;
                    case A2:
                        if (runtimeGb.milliseconds() >= 750) {
                            A2 = "A * * * * * *";
                            runtimeGb.reset();
                            currentGalagaL = GalagaL.B2;
                        }
                        break;
                    case B2:
                        if (runtimeGb.milliseconds() >= 750) {
                            A2 = "* A * * * * *";
                            runtimeGb.reset();
                            currentGalagaL = GalagaL.C2;
                        }
                        break;
                    case C2:
                        if (runtimeGb.milliseconds() >= 750) {
                            A2 = "* * A * * * *";
                            runtimeGb.reset();
                            currentGalagaL = GalagaL.D2;
                        }
                        break;
                    case D2:
                        if (runtimeGb.milliseconds() >= 750) {
                            A2 = "* * * A * * *";
                            runtimeGb.reset();
                            currentGalagaL = GalagaL.E2;
                        }
                        break;
                    case E2:
                        if (runtimeGb.milliseconds() >= 750) {
                            A2 = "* * * * A * *";
                            runtimeGb.reset();
                            currentGalagaL = GalagaL.F2;
                        }
                        break;
                    case F2:
                        if (runtimeGb.milliseconds() >= 750) {
                            A2 = "* * * * * A *";
                            runtimeGb.reset();
                            currentGalagaL = GalagaL.G2;
                        }
                        break;
                    case G2:
                        if (runtimeGb.milliseconds() >= 750) {
                            A2 = "* * * * * * A";
                            runtimeGb.reset();
                            currentGalagaL = GalagaL.H2;
                        }
                        break;
                    case H2:
                        if (runtimeGb.milliseconds() >= 750) {
                            A2 = "* * * * * A *";
                            runtimeGb.reset();
                            currentGalagaL = GalagaL.I2;
                        }
                        break;
                    case I2:
                        if (runtimeGb.milliseconds() >= 750) {
                            A2 = "* * * * A * *";
                            runtimeGb.reset();
                            currentGalagaL = GalagaL.J2;
                        }
                        break;
                    case J2:
                        if (runtimeGb.milliseconds() >= 750) {
                            A2 = "* * * A * * *";
                            runtimeGb.reset();
                            currentGalagaL = GalagaL.K2;
                        }
                        break;
                    case K2:
                        if (runtimeGb.milliseconds() >= 750) {
                            A2 = "* * A * * * *";
                            runtimeGb.reset();
                            currentGalagaL = GalagaL.L2;
                        }
                        break;
                    case L2:
                        if (runtimeGb.milliseconds() >= 750) {
                            A2 = "* A * * * * *";
                            runtimeGb.reset();
                            currentGalagaL = GalagaL.A2;
                        }
                        break;
                }
            }else{
                if(runtimeGa.milliseconds() <= 2000){
                    telemetry.addData("D",D2);
                    D2 = "***Game Over***";
                }else{
                    A2 = "* * * * * * *";
                    B2 = "* * * * * * *";
                    C2 = "* * * * * * *";
                    D2 = "* * * * * * *";
                    E2 = "* * * * * * *";
                    F2 = "* * * * * * *";
                    G2 = "* * * A * * *";
                    PointsG = 10;
                    APressedG = false;
                    robot.stop();
                }




            }


        }else{
            Game = "None";
        }
    }






    @Override
    public void stop () {
        //super.stop();
        robot.stop();
    }




    private enum line {
        A,
        B,
        C,
        D,
        E,
        F,
        G,
        A0,
        B0,
        C0,
        D0,
        E0,
        F0,
        G0
    }


    private enum GalagaL {
        Sa1,
        A2,
        B2,
        C2,
        D2,
        E2,
        F2,
        G2,
        H2,
        I2,
        J2,
        K2,
        L2
    }


    private enum smile {
        F1,
        F2,
        F3,
        F4,
        F5,
        F6
    }


}