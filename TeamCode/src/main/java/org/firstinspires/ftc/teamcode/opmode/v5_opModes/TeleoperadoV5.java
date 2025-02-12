package org.firstinspires.ftc.teamcode.opmode.v5_opModes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.agregadoras.agregadorasRobo.V5;
import org.firstinspires.ftc.teamcode.agregadoras.agregadorasRobo.V5Modes;
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

    private double delayMode = 0.25, cooldownMode = 0;



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
        robot.outtakeIntakeSuperior.goToInitial(robot.carteiro, getRuntime());
    }
    @Override
    public void loop() {

        this.gerenciarModo(robot,gamepadEx2);

        this.robotCentricDrive(robot,gamepadEx1);
        if(robot.v5Modes == V5Modes.SPECIMEN){
            this.bindsChamber(robot,gamepadEx2, robot.carteiro);
        } else if (robot.v5Modes == V5Modes.SAMPLE) {
            this.bindsSample(robot,gamepadEx2,robot.carteiro);
        }
        this.linearHorizontal(robot.intakeInferior.horizontalInferior,gamepadEx2);
        this.linearVertical(robot.outtakeIntakeSuperior.linearVertical,gamepadEx2, robot.carteiro);
        this.bracoGarra(robot.outtakeIntakeSuperior.braco,gamepadEx2,robot.carteiro);
        this.IntakeSuccao(robot.intakeInferior.intakeSuccao,robot.carteiro,gamepadEx2);
        this.garraSuperior(robot.outtakeIntakeSuperior.garraSuperior,robot.carteiro,gamepadEx2);

        this.runActions(robot.carteiro);
        this.testar1servo(robot,gamepadEx1,robot.carteiro);
        this.fullAutoOuttakeChamber(robot,gamepadEx1,robot.carteiro);

        telemetry.addData("MODO DE PONTUAÇÃO", robot.v5Modes);
        //telemetry.addData("Horizontal superior", robot.outtakeIntakeSuperior.horizontalSuperior.servoLinearHorizontal.getPosition());
        //telemetry.addData("braco garra superior", robot.outtakeIntakeSuperior.braco.bracoGarraSuperiorServo.getPosition());
        //telemetry.addData("garra angulação inferior", robot.intakeInferior.garraInferior.angulacaoGarraServo.getPosition());
        //telemetry.addData("Garra port", robot.intakeInferior.garraInferior.servoAberturaDaGarra.getPortNumber());
        //telemetry.addData("Garra pwm status", robot.intakeInferior.garraInferior.servoAberturaDaGarra.getController().getPwmStatus());
        //telemetry.addData("Posição garra", robot.outtakeIntakeSuperior.horizontalSuperior.servoLinearHorizontal.getPosition());
        //telemetry.addData("posicao servo angulacao garra", robot.outtakeIntakeSuperior.garraSuperior.servoAngulacaoGarra.getPosition());
        //telemetry.addData("PWM",robot.outtakeIntakeSuperior.garraSuperior.servoAberturaDaGarra.getController().getPwmStatus());
        //telemetry.addData("get Position",robot.outtakeIntakeSuperior.garraSuperior.servoAberturaDaGarra.getPosition());
        //telemetry.addData("Yaw",-robot.md.lazyImu.get().getRobotYawPitchRollAngles().getYaw());
        //telemetry.addData("Eixo X",robot.md.pose.position.x);
        //telemetry.addData("Eixo Y",robot.md.pose.position.y);
        // telemetry.addData("alpha", robot.intakeInferior.intakeSuccao.colorSensorSugar.getAlpha());
        //telemetry.addData("red", robot.intakeInferior.intakeSuccao.colorSensorSugar.getRed());
        //telemetry.addData("blue", robot.intakeInferior.intakeSuccao.colorSensorSugar.getBlue());

        telemetry.update();


    }

    @NonNull
    private V5 createRobot(HardwareMap hardwareMap) {

        return new V5(hardwareMap, telemetry);

    }
    private void robotCentricDrive(V5 robot,GamepadEx gamepad) {
        //todo: botao para andar diagonal sem alteração no heading
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

    private void bindsChamber(V5 robot, GamepadEx gamepad, OrdersManager carteiro) {

        if(gamepad.getButton(GamepadKeys.Button.A)){
            robot.intakeInferior.goToInitial(carteiro,getRuntime());
            robot.outtakeIntakeSuperior.goToIntakeCHAMBER(carteiro, getRuntime());
        }
        /*if(gamepad.getButton(GamepadKeys.Button.A)){

            carteiro.addOrder(robot.outtakeIntakeSuperior.braco.timeActionBracoGarraSuperior(BracoGarraSuperiorStates.OUTTAKE),0.4,"nome",getRuntime());
        }*/
        if(gamepad.getButton(GamepadKeys.Button.X)){
            robot.intakeInferior.gerenciadorIntake(carteiro,getRuntime());
            //carteiro.addOrder(robot.DiseablePSESuperior(robot,getRuntime()),0.4,"nome",getRuntime());

        }
        // todo: grafico cortado por tempo -> media do sensor
        // todo: automatizar o retorno do intake
        if(gamepad.getButton(GamepadKeys.Button.Y)){
            robot.intakeInferior.goToInitial(carteiro,getRuntime());
            robot.outtakeIntakeSuperior.goToOuttakeCHAMBER(carteiro,getRuntime());
        }

        if(gamepad.getButton(GamepadKeys.Button.B)){
            //ejeting sugar
          robot.intakeInferior.goToInitial(carteiro,getRuntime());
          robot.outtakeIntakeSuperior.goToInitial(robot.carteiro, getRuntime());

            //ejeting transfer
            /*robot.intakeInferior.goToInitial(carteiro,getRuntime());
            //  wait
            robot.outtakeIntakeSuperior.goToOuttakeTransfer(carteiro,getRuntime(),2.0);*/
        }

    }//todo okey
    private void bindsSample(V5 robot, GamepadEx gamepad, OrdersManager carteiro){
        if(gamepad.getButton(GamepadKeys.Button.B)){
            robot.outtakeIntakeSuperior.goToReadyTransfer(carteiro,getRuntime());
        }
        if(gamepad.getButton(GamepadKeys.Button.A)){
            robot.outtakeIntakeSuperior.goToTransfer(carteiro,getRuntime());
            robot.intakeInferior.goToTransfer(carteiro,getRuntime());
        }
        if(gamepad.getButton(GamepadKeys.Button.X)){
            robot.intakeInferior.gerenciadorIntake(carteiro,getRuntime());
        }
        if(gamepad.getButton(GamepadKeys.Button.Y)){
            robot.outtakeIntakeSuperior.goToTransfer(carteiro,getRuntime());
            robot.outtakeIntakeSuperior.goToOuttakeBASKET(carteiro,getRuntime());
        }
    }//todo errado

    private void linearVertical(LinearVertical vertical,GamepadEx gamepad, OrdersManager carteiro)  {
        vertical.PIDF();
        vertical.monitor(telemetry);
        if(gamepad.getButton(GamepadKeys.Button.DPAD_UP)){
            vertical.upSetPoint();
        } else if (gamepad.getButton(GamepadKeys.Button.DPAD_DOWN)) {
            vertical.downSetPoint();
        }
    }//todo okey
    private void bracoGarra(BracoGarraSuperior braco, GamepadEx gamepad, OrdersManager carteiro)  {
        if(gamepad.getLeftY() > 0){
            robot.outtakeIntakeSuperior.braco.upSetPoint(gamepad.getLeftY());
        }
        else if(gamepad.getLeftY() < 0){
            robot.outtakeIntakeSuperior.braco.downSetPoint(gamepad.getLeftY());
        }
    }
    private void linearHorizontal(LinearHorizontalInferior horizontal, GamepadEx gamepad)  {
        if(gamepad.getButton(GamepadKeys.Button.DPAD_LEFT)){
            horizontal.goToExtended();
        }
        else if (gamepad.getButton(GamepadKeys.Button.DPAD_RIGHT)){
            horizontal.goToRetracted();
        }
    }
    private void garraSuperior(GarraSuperior garra, OrdersManager carteiro, GamepadEx gamepad){
        if (gamepad.getButton(GamepadKeys.Button.RIGHT_BUMPER)){
            carteiro.addOrder(robot.outtakeIntakeSuperior.garraSuperior.gerenciadorDoFechamentoDaGarraNoTeleop(getRuntime()),0.0,"garra superior",getRuntime());

        }
        if(gamepad.getRightY() > 0){
            garra.upSetPoint(gamepad.getRightY());
        }
        else if(gamepad.getRightY() < 0){
            garra.downSetPoint(gamepad.getRightY());
        }

        if(gamepad.getRightX() > 0.5){
            garra.upSetPointRot(gamepad.getRightX());
        }
        else if(gamepad.getRightX() < -0.5  ){
            garra.downSetPointRot(gamepad.getRightX());
        }
    }
    private void IntakeSuccao(IntakeSuccao intakeSuccao, OrdersManager carteiro, GamepadEx gamepad){
        if(gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0){
            intakeSuccao.sugador.setPower(IntakeSuccao.power_Sugador * gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));
        } else if (gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0) {
            intakeSuccao.sugador.setPower(IntakeSuccao.power_Sugador * -gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));
        }
        else {intakeSuccao.sugador.setPower(0);}
    }//todo okey
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
    private void gerenciarModo(V5 robot,GamepadEx gamepadEx) {
        // todo: this may cause issues, needs a delay to implement right bc of the velocity of loop vs human input time
        // tudú: dhí´ss mei cóz ichues
        if (getRuntime() >= cooldownMode) {
            cooldownMode = getRuntime() + delayMode;

            if (gamepadEx.getButton(GamepadKeys.Button.START) && this.robot.v5Modes == V5Modes.SPECIMEN) {
                this.robot.v5Modes = V5Modes.SAMPLE;
            }
            else if (gamepadEx.getButton(GamepadKeys.Button.START) && this.robot.v5Modes == V5Modes.SAMPLE) {
                this.robot.v5Modes = V5Modes.SPECIMEN;
            }

        }
    }
    private void runActions(OrdersManager carteiro) {
        carteiro.checkIfCanRun(getRuntime());
        carteiro.runTeleopActions(getRuntime());
    }

}