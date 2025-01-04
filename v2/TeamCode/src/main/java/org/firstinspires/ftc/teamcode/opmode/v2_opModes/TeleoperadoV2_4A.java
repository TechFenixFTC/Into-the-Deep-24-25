package org.firstinspires.ftc.teamcode.opmode.v2_opModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.agregadoras.V2;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name="Teleoperado V2")
public class TeleoperadoV2_4A extends OpMode {

    double garraAngpos = 0.5, garraPos = 0.5;
    V2 robot;

    private double modeCooldown = 0;
    private boolean mode = false;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    private List<Action> runningActions = new ArrayList<>();
    int cond = 0;
     public Servo angulo;
     public  Servo garra;
     public double drive = 0, strafe = 0, turn = 0;
    // public CRServo crServo example;
    public double quandoGuardarObraco = 0, quandoRetrairOLinearHorizontal = 0;
    public boolean precisoGuardarOBraco = false, precisoRetrairOLinearHorizontal = false, precisoDescerOLinear = false;

    double cooldownAnguloGarra = getRuntime(), cooldownGarra = getRuntime(), cooldownLinear = 0;
    GamepadEx gamepadEx1;
    //rampa de aceleração
        double CurrentSpeed;
        double MaxSpeed= 1;
        double SmAcceleration = 0.04;

                boolean anguloGarraAction = false, garraAction = false;

    @Override
    public  void  init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = createRobot(hardwareMap);
        gamepadEx1 = new GamepadEx(gamepad2);

        robot.linearHorizontal.targetPositionBracoDaGarra = 0.50;
        robot.linearHorizontal.servoBracoDaGarra.setPosition(0.50);
    }

    @Override
    public void  loop() {

        if(gamepad1.start) {
            changeMode();
        }
        /*============*\
        |    Chassi     |
        \*============*/
        robotCentricDrive(robot);
        telemetry.addData("X", robot.md.pose.position.x);
        telemetry.addData("Y", robot.md.pose.position.y);


        /*=======================*\
        |    Linear Vertical      |
        \*======================*/
        if (gamepadEx1.getButton(GamepadKeys.Button.BACK) ) {
            robot.linearVertical.reset();
            robot.linearVertical.targetPosition = 0;
        }

        if (gamepadEx1.getButton(GamepadKeys.Button.A))          {

            robot.linearVertical.targetPosition = 10;
            robot.linearHorizontal.targetPositionBracoDaGarra = 0.50;
            robot.linearHorizontal.servoBracoDaGarra.setPosition(0.50);
            precisoRetrairOLinearHorizontal = true;
            quandoRetrairOLinearHorizontal = getRuntime() + 0.3;

        }
        if (gamepadEx1.getButton(GamepadKeys.Button.B))          {

            robot.linearVertical.targetPosition = 300;
            robot.linearHorizontal.targetPositionBracoDaGarra = 0.38;
            robot.linearHorizontal.servoBracoDaGarra.setPosition(0.38);
            precisoRetrairOLinearHorizontal = true;
            quandoRetrairOLinearHorizontal = getRuntime() + 0.2;
            precisoDescerOLinear = true;
            cooldownLinear = getRuntime() + 0.7;

        }


        if (gamepadEx1.getButton(GamepadKeys.Button.X))          {

            robot.linearVertical.targetPosition = 500;
            robot.linearHorizontal.extender(getRuntime());
            precisoGuardarOBraco = true;
            quandoGuardarObraco = getRuntime() + 0.4;

        }

        if (gamepadEx1.getButton(GamepadKeys.Button.Y))          {

            robot.linearHorizontal.targetPositionBracoDaGarra = 0.10;
            robot.linearHorizontal.servoBracoDaGarra.setPosition(robot.linearHorizontal.targetPositionBracoDaGarra);
            precisoRetrairOLinearHorizontal = true;
            quandoRetrairOLinearHorizontal = getRuntime() + 0.3;
            robot.linearVertical.targetPosition = 3500;
         }

        if (gamepadEx1.getButton(GamepadKeys.Button.DPAD_UP))    { robot.linearVertical.upSetPoint();             }

        if (gamepadEx1.getButton(GamepadKeys.Button.DPAD_DOWN) ) { robot.linearVertical.downSetPoint();           }

        robot.linearVertical.PIDF(telemetry);

        telemetry.addData("Posição Linear Vertical motorR", robot.linearVertical.motorR.getCurrentPosition());
        telemetry.addData("Posição Linear Vertical motorL", robot.linearVertical.motorL.getCurrentPosition());
        telemetry.addData("Posição Alvo Linear Vertical", robot.linearVertical.targetPosition);

        if ( precisoGuardarOBraco && getRuntime() >= quandoGuardarObraco) {
            robot.linearHorizontal.targetPositionBracoDaGarra = 0.8;
            robot.linearHorizontal.servoBracoDaGarra.setPosition(0.8);
            precisoGuardarOBraco = false;
        }

        if ( precisoRetrairOLinearHorizontal && getRuntime() >= quandoRetrairOLinearHorizontal) {
            robot.linearHorizontal.recolher(getRuntime());
            precisoRetrairOLinearHorizontal = false;
        }


        /*=======================*\
        |    Linear Horizonal     |
        \*======================*/

        if(gamepad2.dpad_left) {
            if(robot.linearHorizontal.targetPositionBracoDaGarra < 0.99){

                robot.linearHorizontal.targetPositionBracoDaGarra += 0.01;
                robot.linearHorizontal.servoBracoDaGarra.setPosition(robot.linearHorizontal.targetPositionBracoDaGarra);

            }
        }
        if(gamepad2.dpad_right) {
            if(robot.linearHorizontal.targetPositionBracoDaGarra > 0.05) {
                robot.linearHorizontal.targetPositionBracoDaGarra -= 0.01;
                robot.linearHorizontal.servoBracoDaGarra.setPosition(robot.linearHorizontal.targetPositionBracoDaGarra);
            }
        }

        telemetry.addData("Posição rotar braco da garra", robot.linearHorizontal.targetPositionBracoDaGarra);
        telemetry.addData("Posição Alvo Linear Horizontal", robot.linearHorizontal.targetPositionLinearHorizontal);

        if( Math.abs(gamepadEx1.getRightX() ) > 0.1) {
            robot.linearHorizontal.targetPositionLinearHorizontal +=  (-gamepadEx1.getRightX())/50;
            robot.linearHorizontal.servoLinearHorizonal.setPosition(robot.linearHorizontal.targetPositionLinearHorizontal);
        }

        robot.linearHorizontal.handleHorizontalTeleop(getRuntime());

        /*=============*\
        |     Garra     |
        \*=============*/

        if(gamepadEx1.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
            runningActions.add(new SequentialAction(
                    robot.garra.gerenciadorDoFechamentoDaGarraNoTeleop(getRuntime())
            ));

        }

        if(gamepadEx1.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
            runningActions.add(new SequentialAction(
                    robot.garra.gerenciadorDaRotacaoDaGarraNoTeleop(getRuntime())
            ));

        }

        if ( precisoDescerOLinear && getRuntime() >= cooldownLinear ) {
            precisoDescerOLinear = false;
            robot.linearVertical.targetPosition = 10;

        }


        telemetry.addData("angGarra", garraAngpos);


        TelemetryPacket packet = new TelemetryPacket();
        // update running actions
        List<Action> newActions = new ArrayList<>();
        for (Action action : runningActions) {
            action.preview(packet.fieldOverlay());
            if (action.run(packet)) {
                newActions.add(action);
            }
        }
        runningActions = newActions;

        dashboard.sendTelemetryPacket(packet);
        telemetry.update();
    }




    private V2 createRobot(HardwareMap hardwareMap) {


        V2 robot = new V2(hardwareMap, telemetry);
        MecanumDrive.PARAMS.maxProfileAccel = 30;
        
        return  robot;

    }

    private void robotCentricDrive(V2 robot)  {

            drive = Range.clip(-gamepad1.left_stick_y*CurrentSpeed, -1, 1);
            //if( Math.abs(drive) < 0.8 && Math.abs(drive) > 0.02  ) drive = (drive / 1.5);

            strafe = Range.clip(-gamepad1.left_stick_x, -1, 1);
            if ( gamepad1.right_stick_button ) strafe = -0.6;
            if ( gamepad1.left_stick_button ) strafe = 0.6;
            //if ( Math.abs(strafe) < 0.8 && Math.abs(strafe) > 0.02 ) strafe = (strafe / 2.5);

            turn = -gamepad1.right_stick_x;
            if ( gamepad1.left_trigger > 0 ) turn = -gamepad1.left_trigger;
            if ( gamepad1.right_trigger > 0 ) turn = gamepad1.left_trigger;
            turn =   Range.clip(turn / 1.3, -0.7, 0.7);

            robot.md.setDrivePowers(new PoseVelocity2d(new Vector2d(drive, strafe), turn));
            //Ramp


            if(Math.abs(gamepad1.left_stick_y) >0.1 ) {
                if (CurrentSpeed < MaxSpeed) {
                    CurrentSpeed += SmAcceleration;
                }
                CurrentSpeed = Math.min(CurrentSpeed, MaxSpeed);
            }
            else{
                    if (CurrentSpeed > 0.0) {
                        CurrentSpeed = 0;
                    }
                }


            robot.md.updatePoseEstimate();
            telemetry.addData("currentspeed", CurrentSpeed);
            telemetry.addData("Heading", Math.toDegrees(robot.md.pose.heading.real));
            telemetry.addData("power do motor",robot.md.leftBack.getPower());
    }

    private void changeMode() {
        if(getRuntime() < modeCooldown) {
            return;
        }
        modeCooldown = getRuntime() + 0.2;

        if (mode) {
            mode = false;
            gamepadEx1 = new GamepadEx(gamepad2);
            return;
        }

        mode = true;
        gamepadEx1 = new GamepadEx(gamepad1);

    }
}

