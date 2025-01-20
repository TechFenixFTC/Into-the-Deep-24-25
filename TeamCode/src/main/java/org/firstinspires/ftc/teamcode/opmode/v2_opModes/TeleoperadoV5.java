package org.firstinspires.ftc.teamcode.opmode.v2_opModes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.agregadoras.V2;
import org.firstinspires.ftc.teamcode.agregadoras.Subsistemas;
import org.firstinspires.ftc.teamcode.subsystems.OrdersManager;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasInferiores.Garra.GarraInferior;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasInferiores.Horizontal.LinearHorizontalInferior;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasSuperiores.BracoGarraMotor.BracoGarraV4;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasSuperiores.Garra.GarraSuperior;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasSuperiores.Horizontal.LinearHorizontalSuperior;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasSuperiores.LinearVertical.LinearVertical;

@TeleOp(name="Teleoperado V5")
public class TeleoperadoV5 extends OpMode {


    private V2 robot;
    GamepadEx gamepadEx1,gamepadEx2;
    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private Telemetry dashboardTelemetry = dashboard.getTelemetry();


    @Override
    public  void init() {

        robot = this.createRobot(hardwareMap);
        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);
        robot.intakeOutake.RobotState = Subsistemas.vertexState.Initial;



    }
    @Override
    public void loop() {
        this.gerenciarModo(robot,gamepadEx1);
        this.robotCentricDrive(robot,gamepadEx1);
        this.binds(robot,gamepadEx2);
        this.linearHorizontalInferior(robot.intakeOutake.horizontalInferior,gamepadEx1);
        this.linearHorizontalSuperior(robot.intakeOutake.horizontalSuperior,gamepadEx1);
        this.linearVertical(robot.intakeOutake.linearVertical,gamepadEx1);
        this.bracoGarra(robot.intakeOutake.braco,gamepadEx1);
        this.garraInferior(robot.intakeOutake.garraInferior,robot.intakeOutake.carteiro,gamepadEx1);
        this.garraSuperior(robot.intakeOutake.garraSuperior,robot.intakeOutake.carteiro,gamepadEx1);
        this.runActions(robot.intakeOutake.carteiro);
    }

    @NonNull
    private V2 createRobot(HardwareMap hardwareMap) {

        return new V2(hardwareMap, telemetry);

    }
    private void robotCentricDrive(V2 robot,GamepadEx gamepad)  {

    }
    private void binds(V2 robot, GamepadEx gamepad) {


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
    private void garraInferior(GarraInferior garra,OrdersManager carteiro,GamepadEx gamepad){
        if(gamepad.getButton(GamepadKeys.Button.A)){
            robot.intakeOutake.garraInferior.goToIntake(0);
        }

    }

    private void gerenciarModo(V2 robot,GamepadEx gamepad) {


    }
    private void runActions(OrdersManager carteiro) {

        carteiro.checkIfCanRun(getRuntime());
        carteiro.runTeleopActions(getRuntime());
        carteiro.removeOrderByName(robot.intakeOutake.getRobotState().name());

    }

}