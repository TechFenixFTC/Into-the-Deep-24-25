package org.firstinspires.ftc.teamcode.opmode.v2_opModes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.agregadoras.agregadorasRobo.V5;
import org.firstinspires.ftc.teamcode.agregadoras.agregadorasSubsistemas.Inferior.UnderGrounSubystemStates;
import org.firstinspires.ftc.teamcode.agregadoras.agregadorasSubsistemas.Superior.UpperSubsystemStates;
import org.firstinspires.ftc.teamcode.subsystems.OrdersManager;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasInferiores.Horizontal.LinearHorizontalInferior;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasInferiores.Sugar.IntakeSuccao;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasSuperiores.BracoGarra.BracoGarraSuperior;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasSuperiores.Garra.GarraSuperior;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasSuperiores.LinearVertical.LinearVertical;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name="Teleoperado V5")
public class TeleoperadoV5 extends OpMode {

    List<Servo> servos = new ArrayList<>(4);
    private V5 robot;
    GamepadEx gamepadEx1,gamepadEx2;
    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private Telemetry dashboardTelemetry = dashboard.getTelemetry();



    public boolean needToOutake = false;

    @Override
    public  void init() {

        robot = this.createRobot(hardwareMap);
        Pose2d initialPose = new Pose2d(38,-60 , Math.toRadians(-90));
        robot.md.pose = initialPose;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);
        robot.intakeInferior.underGrounSubystemStates = UnderGrounSubystemStates.TRASNFER;
        robot.outtakeIntakeSuperior.upperSubsystemStates = UpperSubsystemStates.INITIAL;
        robot.intakeInferior.goToInitial(robot.carteiro,getRuntime());
    }
    @Override
    public void loop() {

        this.gerenciarModo(robot,gamepadEx1);
        this.robotCentricDrive(robot,gamepadEx1);
        this.binds(robot,gamepadEx2,robot.carteiro,gamepadEx1);
        this.linearHorizontalInferior(robot.intakeInferior.horizontalInferior,gamepadEx1);
        this.linearVertical(robot.outtakeIntakeSuperior.linearVertical,gamepadEx1, robot.carteiro);
        this.bracoGarra(robot.outtakeIntakeSuperior.braco,gamepadEx1,robot.carteiro);
        this.IntakeSuccao(robot.intakeInferior.intakeSuccao,robot.carteiro,gamepadEx2);
        this.garraSuperior(robot.outtakeIntakeSuperior.garraSuperior,robot.carteiro,gamepadEx1);
        this.runActions(robot.carteiro);
        this.testar1servo(robot,gamepadEx1,robot.carteiro);
        this.fullAutoOuttakeChamber(robot,gamepadEx1,robot.carteiro);
        this.setPositionServos();

        //telemetry.addData("Horizontal superior", robot.outtakeIntakeSuperior.horizontalSuperior.servoLinearHorizontal.getPosition());
        //telemetry.addData("braco garra superior", robot.outtakeIntakeSuperior.braco.bracoGarraSuperiorServo.getPosition());
        //telemetry.addData("garra angulação inferior", robot.intakeInferior.garraInferior.angulacaoGarraServo.getPosition());
        //telemetry.addData("Garra port", robot.intakeInferior.garraInferior.servoAberturaDaGarra.getPortNumber());
        //telemetry.addData("Garra pwm status", robot.intakeInferior.garraInferior.servoAberturaDaGarra.getController().getPwmStatus());
        //telemetry.addData("Posição garra", robot.outtakeIntakeSuperior.horizontalSuperior.servoLinearHorizontal.getPosition());
        telemetry.addData("posicao servo angulacao garra", robot.outtakeIntakeSuperior.garraSuperior.servoAngulacaoGarra.getPosition());
        telemetry.addData("PWM",robot.outtakeIntakeSuperior.garraSuperior.servoAberturaDaGarra.getController().getPwmStatus());
        telemetry.addData("get Position",robot.outtakeIntakeSuperior.garraSuperior.servoAberturaDaGarra.getPosition());
        telemetry.addData("Yaw",-robot.md.lazyImu.get().getRobotYawPitchRollAngles().getYaw());
        telemetry.addData("Eixo X",robot.md.pose.position.x);
        telemetry.addData("Eixo Y",robot.md.pose.position.y);
        telemetry.addData("alpha", robot.intakeInferior.intakeSuccao.colorSensorSugar.getAlpha());
        telemetry.addData("red", robot.intakeInferior.intakeSuccao.colorSensorSugar.getRed());
        telemetry.addData("blue", robot.intakeInferior.intakeSuccao.colorSensorSugar.getBlue());

        telemetry.update();


    }

    @NonNull
    private V5 createRobot(HardwareMap hardwareMap) {

        return new V5(hardwareMap, telemetry);

    }
    private void robotCentricDrive(V5 robot,GamepadEx gamepad) {
        robot.md.updatePoseEstimate();
        double drive = Range.clip(gamepad.getLeftY(), -1, 1);

        //if( Math.abs(drive) < 0.8 && Math.abs(drive) > 0.02  ) drive = (drive / 1.5);

        double strafe = Range.clip(-gamepad.getLeftX(), -1, 1);
        if (gamepad1.right_stick_button) strafe = -0.6;
        if (gamepad1.left_stick_button) strafe = 0.6;
        //if ( Math.abs(strafe) < 0.8 && Math.abs(strafe) > 0.02 ) strafe = (strafe / 2.5);

        double turn = -gamepad.getRightX();
        if (gamepad1.left_trigger > 0) {
            turn = -gamepad1.left_trigger * 0.5;
        }

        if (gamepad1.right_trigger > 0) {
            turn = gamepad1.right_trigger * 0.5;
        }
        if(strafe != 0 && turn == 0){

        }

        turn = Range.clip(turn / 1.3, -0.7, 0.7);

        robot.md.setDrivePowers(new PoseVelocity2d(new Vector2d(drive, strafe), turn));



    }

    private void testar1servo(V5 robot, GamepadEx gamepad,OrdersManager carteiro) {
    }

    private void binds(V5 robot, GamepadEx gamepad, OrdersManager carteiro, GamepadEx gamepadEx1) {

        // X -> A -> B -> Y-> DPAD RIGHT
        if(gamepad.getButton(GamepadKeys.Button.A)){
            robot.outtakeIntakeSuperior.goToIntakeCHAMBER(carteiro, getRuntime());

        }
        if(gamepad.getButton(GamepadKeys.Button.B)){
            robot.outtakeIntakeSuperior.goToOuttakeCHAMBER(carteiro,getRuntime());

        }
        if(gamepad.getButton(GamepadKeys.Button.X)){
            robot.intakeInferior.goToIntake(carteiro,getRuntime());
        }
        if(gamepad.getButton(GamepadKeys.Button.Y)){
            robot.intakeInferior.goToInitial(carteiro,getRuntime());
        }

        if (gamepad.getButton(GamepadKeys.Button.RIGHT_BUMPER)){
            carteiro.addOrder(robot.outtakeIntakeSuperior.garraSuperior.gerenciadorDoFechamentoDaGarraNoTeleop(getRuntime()),0.0,"garra superior",getRuntime());

        }
        if(gamepadEx1.getButton(GamepadKeys.Button.LEFT_BUMPER)){
            robot.intakeInferior.goToTransfer(carteiro,getRuntime());
            robot.outtakeIntakeSuperior.goToTransfer(carteiro,getRuntime());

        }


    }

    private void linearVertical(LinearVertical vertical,GamepadEx gamepad, OrdersManager carteiro)  {
        vertical.PIDF();
       // todo: controle manual
        if(gamepad.getButton(GamepadKeys.Button.DPAD_UP)){
            vertical.upSetPoint();
        }
        if(gamepad.getButton(GamepadKeys.Button.DPAD_DOWN)){
            vertical.downSetPoint();
        }

        if(gamepad.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON)){
            carteiro.addOrder(vertical.ElevadorGoTo(1380), 0, "SKY-LINE", getRuntime());
        }
        if(gamepad.getButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)){
            carteiro.addOrder(vertical.ElevadorGoTo(50), 0, "FLOOR-LINE", getRuntime());
        }

    }
    private void bracoGarra(BracoGarraSuperior braco, GamepadEx gamepad, OrdersManager carteiro)  {
    }
    private void linearHorizontalInferior(LinearHorizontalInferior horizontal, GamepadEx gamepad)  {

    }
    private void garraSuperior(GarraSuperior garra, OrdersManager carteiro, GamepadEx gamepad){
    }
    private void IntakeSuccao(IntakeSuccao subsistemasInferiores, OrdersManager carteiro, GamepadEx gamepad){
        if(gamepad.getButton(GamepadKeys.Button.DPAD_UP)){
            robot.intakeInferior.intakeSuccao.sugador.setPower(IntakeSuccao.power_Sugador);
        } else if (gamepad.getButton(GamepadKeys.Button.DPAD_DOWN)) {
            robot.intakeInferior.intakeSuccao.sugador.setPower(IntakeSuccao.power_Sugador * -1);
        }
    }
    private void setPositionServos(){
    }

    private void fullAutoOuttakeChamber(V5 robot , GamepadEx gamepad,OrdersManager carteiro){
        /*if(gamepad.getButton(GamepadKeys.Button.Y)){
            robot.md.updatePoseEstimate();
            Actions.runBlocking(
                new SequentialAction(

                        new ParallelAction(

                            //robot.outtakeIntakeSuperior.goToOuttakeCHAMBER(),
                            robot.MoveOuttake(robot)

                    ),
                    robot.outtakeIntakeSuperior.garraSuperior.abrirGarra()

                )
            );
        }
        if(gamepad.getButton(GamepadKeys.Button.X)){
            robot.md.updatePoseEstimate();
            Actions.runBlocking(
                    new SequentialAction(
                            robot.outtakeIntakeSuperior.goToIntakeChamber2(),
                            robot.MoveIntake(robot)
                    )
            );
        }*/

    }

    private void gerenciarModo(V5 robot,GamepadEx gamepad) {


    }
    private void runActions(OrdersManager carteiro) {
        carteiro.checkIfCanRun(getRuntime());
        carteiro.runTeleopActions(getRuntime());
    }

}