package org.firstinspires.ftc.teamcode.opmode.v2_opModes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;

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
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasSuperiores.BracoGarra.BracoGarraV4;
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
        this.binds(robot,gamepadEx2,robot.intakeInferior.carteiro);
        this.linearHorizontalInferior(robot.intakeInferior.horizontalInferior,gamepadEx1);
        this.linearHorizontalSuperior(robot.outtakeIntakeSuperior.horizontalSuperior,gamepadEx1);
        this.linearVertical(robot.outtakeIntakeSuperior.linearVertical,gamepadEx1);
        this.bracoGarra(robot.outtakeIntakeSuperior.braco,gamepadEx1);
        this.garraInferior(robot.intakeInferior.garraInferior,robot.intakeInferior.carteiro,gamepadEx1);
        this.garraSuperior(robot.outtakeIntakeSuperior.garraSuperior,robot.outtakeIntakeSuperior.carteiro,gamepadEx1);
        this.runActions(robot.intakeInferior.carteiro);


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
            carteiro.addOrder(robot.intakeInferior.goToReadyToIntake(),0.0,"goToReady");
        }

        if(gamepad.getButton(GamepadKeys.Button.A)){
           carteiro.addOrder(robot.intakeInferior.goToIntake(),0.0,"goToIntake");
        }
        if(gamepad.getButton(GamepadKeys.Button.B)){
            carteiro.addOrder(robot.intakeInferior.goToTransfer(),0.0,"goToTransfer");
        }
    }

    private void linearVertical(LinearVertical vertical,GamepadEx gamepad)  {
        if(gamepad.getButton(GamepadKeys.Button.DPAD_UP)){
            vertical.upSetPoint();
        }
        if(gamepad.getButton(GamepadKeys.Button.DPAD_DOWN)){
            vertical.downSetPoint();
        }
    }
    private void bracoGarra(BracoGarraV4 braco, GamepadEx gamepad)  {
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
        carteiro.removeOrderByName(robot.intakeInferior.underGrounSubystemStates.name());
    }

}