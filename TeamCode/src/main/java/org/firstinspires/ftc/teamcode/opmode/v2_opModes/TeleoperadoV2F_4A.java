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
    GamepadEx gamepadEx;
    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private Telemetry dashboardTelemetry = dashboard.getTelemetry();


    @Override
    public  void init() {

        robot = this.createRobot(hardwareMap);
        gamepadEx = new GamepadEx(gamepad2);
        robot.intakeOutake.RobotState = Vertex.vertexState.Initial;



    }
    @Override
    public void loop() {
        this.gerenciarModo(robot);
        this.robotCentricDrive(robot);
        this.binds(robot);
        this.linearHorizontal(robot.intakeOutake.linearHorizontal);
        this.linearVertical(robot.intakeOutake.linearVertical);
        this.bracoGarra(robot.intakeOutake.braco);
        this.garra(robot.intakeOutake.garra,robot.intakeOutake.carteiro);
        this.runActions(robot.intakeOutake.carteiro);

        robot.intakeOutake.braco.handleBracoTeleop(getRuntime());

    }

    @NonNull
    private V2 createRobot(HardwareMap hardwareMap) {

        return new V2(hardwareMap, telemetry);

    }
    private void robotCentricDrive(V2 robot)  {

    }
    private void binds(V2 robot) {

        if(gamepadEx.getButton(GamepadKeys.Button.A) || robot.intakeOutake.RobotState == Vertex.vertexState.Intermediate){
            robot.intakeOutake.IntermediatePosition(getRuntime());
            robot.intakeOutake.RobotState = Vertex.vertexState.Intermediate;

        }
        if(gamepadEx.getButton(GamepadKeys.Button.B) ||  robot.intakeOutake.RobotState == Vertex.vertexState.Initial){
            robot.intakeOutake.InitialPosition(getRuntime());
            robot.intakeOutake.RobotState = Vertex.vertexState.Initial;
        }
        if(gamepadEx.getButton(GamepadKeys.Button.X) ||  robot.intakeOutake.RobotState == Vertex.vertexState.Intake){
            robot.intakeOutake.IntakePosition(getRuntime());
            robot.intakeOutake.RobotState = Vertex.vertexState.Intake;
        }
        if(gamepadEx.getButton(GamepadKeys.Button.Y) ||  robot.intakeOutake.RobotState == Vertex.vertexState.Intermediate){
            robot.intakeOutake.OuttakePosition(getRuntime());
            robot.intakeOutake.RobotState = Vertex.vertexState.Outtake;
        }
    }
    private void linearHorizontal(LinearHorizontal horizontal)  {
        if(gamepadEx.getButton(GamepadKeys.Button.DPAD_RIGHT)){
            horizontal.upSetPoint();
        }
        if(gamepadEx.getButton(GamepadKeys.Button.DPAD_LEFT)){
            horizontal.downSetPoint();
        }
    }
    private void linearVertical(LinearVertical vertical)  {
        if(gamepad1.dpad_up){
            vertical.upSetPoint();
        }
        if(gamepad1.dpad_down){
            vertical.downSetPoint();
        }
    }
    private void bracoGarra(BracoGarra braco)  {

        if(gamepadEx.getButton(GamepadKeys.Button.DPAD_DOWN)){
            robot.intakeOutake.braco.downBraco();

        }
        if(gamepadEx.getButton(GamepadKeys.Button.DPAD_UP)){
            robot.intakeOutake.braco.upBraco();
        }


    }
    private void garra(Garra garra,OrdersManager carteiro){

        if(gamepadEx.getButton(GamepadKeys.Button.LEFT_BUMPER)){
            carteiro.addOrder(garra.gerenciadorDoFechamentoDaGarraNoTeleop(getRuntime()),getRuntime(),"abrirFecharGarra");
        }
        if(gamepadEx.getButton(GamepadKeys.Button.RIGHT_BUMPER)){
            carteiro.addOrder(garra.gerenciadorDaRotacaoDaGarraNoTeleop(getRuntime()),getRuntime(),"rotacionarGarra");
        }
    }


    private void gerenciarModo(V2 robot) {


    }
    private void runActions(OrdersManager carteiro) {

        carteiro.checkIfCanRun(getRuntime());
        carteiro.runTeleopActions(getRuntime());
        carteiro.removeOrderByName(robot.intakeOutake.getRobotState().name());

    }

}