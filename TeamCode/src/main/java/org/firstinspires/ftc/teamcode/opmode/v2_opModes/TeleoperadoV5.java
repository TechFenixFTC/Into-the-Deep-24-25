package org.firstinspires.ftc.teamcode.opmode.v2_opModes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.agregadoras.agregadorasRobo.V5;
import org.firstinspires.ftc.teamcode.agregadoras.agregadorasSubsistemas.Inferior.UnderGrounSubystemStates;
import org.firstinspires.ftc.teamcode.subsystems.OrdersManager;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasInferiores.Garra.GarraInferior;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasInferiores.Horizontal.LinearHorizontalInferior;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasSuperiores.BracoGarra.BracoGarraSuperior;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasSuperiores.Garra.GarraSuperior;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasSuperiores.Horizontal.LinearHorizontalSuperior;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasSuperiores.LinearVertical.LinearVertical;

import java.util.ArrayList;

@TeleOp(name="Teleoperado V5")
public class TeleoperadoV5 extends OpMode {


    private V5 robot;
    GamepadEx gamepadEx1,gamepadEx2;
    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private Telemetry dashboardTelemetry = dashboard.getTelemetry();

    ArrayList<Servo> servos = new ArrayList<>();



    @Override
    public  void init() {

        robot = this.createRobot(hardwareMap);
        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);
        robot.intakeInferior.underGrounSubystemStates = UnderGrounSubystemStates.TRASNFER;
    }
    @Override
    public void loop() {
        this.gerenciarModo(robot,gamepadEx1);
        this.robotCentricDrive(robot,gamepadEx1);
        this.binds(robot,gamepadEx2,robot.carteiro);
        this.linearHorizontalInferior(robot.intakeInferior.horizontalInferior,gamepadEx1);
        this.linearHorizontalSuperior(robot.outtakeIntakeSuperior.horizontalSuperior,gamepadEx1);
        this.linearVertical(robot.outtakeIntakeSuperior.linearVertical,gamepadEx1);
        this.bracoGarra(robot.outtakeIntakeSuperior.braco,gamepadEx1,robot.carteiro);
        this.garraInferior(robot.intakeInferior.garraInferior,robot.carteiro,gamepadEx1);
        this.garraSuperior(robot.outtakeIntakeSuperior.garraSuperior,robot.carteiro,gamepadEx1);
        this.runActions(robot.carteiro);


        telemetry.addData("horizontal", robot.intakeInferior.horizontalInferior.servoLinearHorizontal.getPosition());
        telemetry.addData("angulacao", robot.intakeInferior.garraInferior.angulacaoGarraInferiorServo.getPosition());
        telemetry.addData("abertura", robot.intakeInferior.garraInferior.aberturaGarraInferiorServo.getPosition());
        telemetry.addData("rotacao", robot.intakeInferior.garraInferior.rotacaoGarraInferiorServo.getPosition());

        telemetry.update();

    }

    @NonNull
    private V5 createRobot(HardwareMap hardwareMap) {

        return new V5(hardwareMap, telemetry);

    }
    private void robotCentricDrive(V5 robot,GamepadEx gamepad)  {

    }

    private void testar1servo() {

    }
    private void binds(V5 robot, GamepadEx gamepad,OrdersManager carteiro) {
        if(gamepad.getButton(GamepadKeys.Button.X)){
            carteiro.addOrder(robot.intakeInferior.goToReadyToIntake(carteiro),0.0,"goToReady");
        }
        if(gamepad.getButton(GamepadKeys.Button.A)){
           carteiro.addOrder(robot.intakeInferior.goToIntake(carteiro),0.0,"goToIntake");
        }
        if(gamepad.getButton(GamepadKeys.Button.B)){
            carteiro.addOrder(robot.intakeInferior.goToTransfer(carteiro),0.0,"goToTransfer");
           // carteiro.addOrder(robot.outtakeIntakeSuperior.goToTransfer(carteiro),0.4,"goToTransferOuttake");
        }
        if(gamepad.getButton(GamepadKeys.Button.Y)){
            carteiro.addOrder(robot.intakeInferior.goToTransfer(carteiro),0,"goToTransfer");
           carteiro.addOrder(robot.outtakeIntakeSuperior.goToTransfer(carteiro),0.0,"goToOuttake");
           carteiro.addOrder(robot.intakeInferior.garraInferior.goToAbrirGarra(),1,"abrir garra");

        }
        if(gamepad.getButton(GamepadKeys.Button.RIGHT_BUMPER)){
            Actions.runBlocking(
                    robot.intakeInferior.garraInferior.goToAbrirGarra()
            );
        }
        if(gamepad.getButton(GamepadKeys.Button.LEFT_BUMPER)){
            Actions.runBlocking(
                    robot.intakeInferior.garraInferior.fecharGarra()
            );

        }
    }

    private void linearVertical(LinearVertical vertical,GamepadEx gamepad)  {

       // todo: controle manual
        if(gamepad.getButton(GamepadKeys.Button.DPAD_UP)){
            vertical.upSetPoint();
        }
        if(gamepad.getButton(GamepadKeys.Button.DPAD_DOWN)){
            vertical.downSetPoint();
        }
    }
    private void bracoGarra(BracoGarraSuperior braco, GamepadEx gamepad, OrdersManager carteiro)  {
        carteiro.addOrder(robot.outtakeIntakeSuperior.braco.goToInital(), 0,"braco superior inicial");
    }
    private void linearHorizontalSuperior(LinearHorizontalSuperior horizontal, GamepadEx gamepad)  {

    }
    private void linearHorizontalInferior(LinearHorizontalInferior horizontal, GamepadEx gamepad)  {

    }
    private void garraSuperior(GarraSuperior garra, OrdersManager carteiro, GamepadEx gamepad){
    }
    private void garraInferior(GarraInferior subsistemasInferiores, OrdersManager carteiro, GamepadEx gamepad){

    }
    private void gerenciarModo(V5 robot,GamepadEx gamepad) {


    }
    private void runActions(OrdersManager carteiro) {
        carteiro.checkIfCanRun(getRuntime());
        carteiro.runTeleopActions(getRuntime());
        //carteiro.removeOrderByName(robot.intakeInferior.underGrounSubystemStates.name());
    }

}