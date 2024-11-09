package org.firstinspires.ftc.teamcode.opmode.v1_opModes;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.agregadoras.V1;
import org.firstinspires.ftc.teamcode.roadrunner.V1_rr.MecanumDrive;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Vertex.Garra;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name="Teleoperado")
public class TeleoperadoV1_4A extends OpMode {

    double garraAngpos = 0.5, garraPos = 0.5;
    V1 robot;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    private List<Action> runningActions = new ArrayList<>();
    int cond = 0;
     public Servo angulo;
     public  Servo garra;
     public double drive = 0, strafe = 0, turn = 0;
    // public CRServo crServo example;
    double cooldownAnguloGarra = getRuntime(), cooldownGarra = getRuntime();
    GamepadEx gamepadEx1;
    boolean anguloGarraAction = false, garraAction = false;

    @Override
    public  void  init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = createRobot(hardwareMap);
        gamepadEx1 = new GamepadEx(gamepad1);

    }

    @Override
    public void  loop() {

        //Pose2d my pose = robot.md.pose;
        //robot.webcam.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
        //telemetry.addData("Vermelho", robot.getPipeOutput());
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

        if (gamepadEx1.getButton(GamepadKeys.Button.A))          { robot.linearVertical.targetPosition = 10;      }

        if (gamepadEx1.getButton(GamepadKeys.Button.X))          { robot.linearVertical.targetPosition = 200;     }

        if (gamepadEx1.getButton(GamepadKeys.Button.Y))          { robot.linearVertical.targetPosition = 1400;    }

        if (gamepadEx1.getButton(GamepadKeys.Button.DPAD_UP))    { robot.linearVertical.upSetPoint();             }

        if (gamepadEx1.getButton(GamepadKeys.Button.DPAD_DOWN) ) { robot.linearVertical.downSetPoint();           }

        robot.linearVertical.PIDF(telemetry);

        telemetry.addData("Posição Linear Vertical motorR", robot.linearVertical.motorR.getCurrentPosition());
        telemetry.addData("Posição Linear Vertical motorL", robot.linearVertical.motorL.getCurrentPosition());
        telemetry.addData("Posição Alvo Linear Vertical", robot.linearVertical.targetPosition);




        /*=======================*\
        |    Linear Horizonal     |
        \*======================*/
        if (gamepadEx1.getButton(GamepadKeys.Button.BACK) ) {
            robot.linearVertical.reset();
            robot.linearVertical.targetPosition = 0;
            robot.linearHorizontal.reset();
            robot.linearVertical.targetPosition = 0;
        }
        if(gamepad1.dpad_left) {
            garraAngpos -= 0.01;
            robot.garra.rotacaoDaGarra.setPosition(garraAngpos);
        }
        if(gamepad1.dpad_right) {
            garraAngpos += 0.01;
            robot.garra.rotacaoDaGarra.setPosition(garraAngpos);
        }
        robot.linearHorizontal.PIDF();

        telemetry.addData("Posição Linear Horizontal", robot.linearHorizontal.getPosition());
        telemetry.addData("Posição Alvo Linear Horizontal", robot.linearHorizontal.targetPosition);


        /*=============*\
        |     Garra     |
        \*=============*/

        if(gamepad1.left_bumper) {
            runningActions.add(new SequentialAction(
                    robot.garra.gerenciadorDoFechamentoDaGarraNoTeleop(getRuntime())
            ));

        }

        if(gamepad1.right_bumper) {
            runningActions.add(new SequentialAction(
                    robot.garra.gerenciadorDaRotacaoDaGarraNoTeleop(getRuntime())
            ));

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




    private V1 createRobot(HardwareMap hardwareMap) {


        V1 robot = new V1(hardwareMap, telemetry);
        MecanumDrive.PARAMS.maxProfileAccel = 30;
        return  robot;

    }

    private void robotCentricDrive(V1 robot)  {

        if(gamepad1.start) {
            robot.updateAutoRoutes();
            robot.md.updatePoseEstimate();

            if (robot.garra.estadoFechamentoDaGarra == Garra.FechamentoDaGarraState.FECHADA) {
                Actions.runBlocking(
                        robot.GoToBasket
                );
            } else if (robot.garra.estadoFechamentoDaGarra == Garra.FechamentoDaGarraState.ABERTA) {
                Actions.runBlocking(
                        robot.GoToSubmersible
                );

            }
        }
        else {
            drive = -gamepad1.left_stick_y;
            //if( Math.abs(drive) < 0.8 && Math.abs(drive) > 0.02  ) drive = (drive / 1.5);

            strafe = -gamepad1.left_stick_x;
            //if( Math.abs(strafe) < 0.8 && Math.abs(strafe) > 0.02 ) strafe = (strafe / 2.5);

            turn = -gamepad1.right_stick_x;
            if( Math.abs(turn) < 0.95 ) turn = turn / 1.5;
            robot.md.setDrivePowers(new PoseVelocity2d(new Vector2d(drive, strafe), turn));

        }

        robot.md.updatePoseEstimate();

        telemetry.addData("Heading", Math.toDegrees(robot.md.pose.heading.real));


    }
}

