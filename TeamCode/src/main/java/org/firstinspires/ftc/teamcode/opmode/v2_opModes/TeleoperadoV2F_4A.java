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
import org.firstinspires.ftc.teamcode.agregadoras.Vertex;
import org.firstinspires.ftc.teamcode.subsystems.OrdersManager;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasSuperiores.BracoGarraMotor.BracoGarra;
import org.firstinspires.ftc.teamcode.subsystems.common.Garra.Garra;
import org.firstinspires.ftc.teamcode.subsystems.common.Horizontal.LinearHorizontal;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasSuperiores.LinearVertical.LinearVertical;

@TeleOp(name="Teleoperado V2f")
public class TeleoperadoV2F_4A extends OpMode {


    private V2 robot;
    GamepadEx gamepadEx1,gamepadEx2;
    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private Telemetry dashboardTelemetry = dashboard.getTelemetry();


    @Override
    public  void init() {

        robot = this.createRobot(hardwareMap);
        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);
        robot.intakeOutake.RobotState = Vertex.vertexState.Initial;



    }
    @Override
    public void loop() {
        this.gerenciarModo(robot,gamepadEx1);
        this.robotCentricDrive(robot,gamepadEx1);
        this.binds(robot,gamepadEx2);
        this.linearHorizontal(robot.intakeOutake.linearHorizontal,gamepadEx1);
        this.linearVertical(robot.intakeOutake.linearVertical,gamepadEx1);
        this.bracoGarra(robot.intakeOutake.braco,gamepadEx1);
        this.garra(robot.intakeOutake.garra,robot.intakeOutake.carteiro,gamepadEx1);
        this.runActions(robot.intakeOutake.carteiro);

        robot.intakeOutake.braco.handleBracoTeleop(getRuntime());

    }

    @NonNull
    private V2 createRobot(HardwareMap hardwareMap) {

        return new V2(hardwareMap, telemetry);

    }
    private void robotCentricDrive(V2 robot,GamepadEx gamepad)  {

    }
    private void binds(V2 robot, GamepadEx gamepad) {

        if(gamepad.getButton(GamepadKeys.Button.A) || robot.intakeOutake.RobotState == Vertex.vertexState.Intermediate){
            robot.intakeOutake.IntermediatePosition(getRuntime());
            robot.intakeOutake.RobotState = Vertex.vertexState.Intermediate;

        }
        if(gamepad.getButton(GamepadKeys.Button.B) ||  robot.intakeOutake.RobotState == Vertex.vertexState.Initial){
            robot.intakeOutake.InitialPosition(getRuntime());
            robot.intakeOutake.RobotState = Vertex.vertexState.Initial;
        }
        if(gamepad.getButton(GamepadKeys.Button.X) ||  robot.intakeOutake.RobotState == Vertex.vertexState.Intake){
            robot.intakeOutake.IntakePosition(getRuntime());
            robot.intakeOutake.RobotState = Vertex.vertexState.Intake;
        }
        if(gamepad.getButton(GamepadKeys.Button.Y) ||  robot.intakeOutake.RobotState == Vertex.vertexState.Intermediate){
            robot.intakeOutake.OuttakePosition(getRuntime());
            robot.intakeOutake.RobotState = Vertex.vertexState.Outtake;
        }
    }
    private void linearHorizontal(LinearHorizontal horizontal,GamepadEx gamepad)  {
        if(gamepad.getButton(GamepadKeys.Button.DPAD_RIGHT)){
            horizontal.upSetPoint();
        }
        if(gamepad.getButton(GamepadKeys.Button.DPAD_LEFT)){
            horizontal.downSetPoint();
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
    private void bracoGarra(BracoGarra braco,GamepadEx gamepad)  {

        if(gamepad.getButton(GamepadKeys.Button.DPAD_DOWN)){
            robot.intakeOutake.braco.downBraco();

        }
        if(gamepad.getButton(GamepadKeys.Button.DPAD_UP)){
            robot.intakeOutake.braco.upBraco();
        }


    }
    private void garra(Garra garra,OrdersManager carteiro,GamepadEx gamepad){

        if(gamepad.getButton(GamepadKeys.Button.LEFT_BUMPER)){
            carteiro.addOrder(garra.gerenciadorDoFechamentoDaGarraNoTeleop(getRuntime()),getRuntime(),"abrirFecharGarra");
        }
        if(gamepad.getButton(GamepadKeys.Button.RIGHT_BUMPER)){
            carteiro.addOrder(garra.gerenciadorDaRotacaoDaGarraNoTeleop(getRuntime()),getRuntime(),"rotacionarGarra");
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