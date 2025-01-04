package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

public class OrdersManager {

    Telemetry telemetry;
    public  OrdersManager(Telemetry telemetry) {
        this.telemetry = telemetry;
    }



    private static class Order {
        Action action;
        double time;

        String name;

        public Order(Action action, double time, String name) {
            this.action = action;
            this.time = time;
            this.name = name;
        }

        public String getName() {
            return name;
        }

        public double getTime() {
            return time;
        }

        public Action getAction() {
            return action;
        }
    }

    private List<Action> runningActions = new ArrayList<>();
    private List<Order> orders = new ArrayList<>();


    public void addOrder(Action action, double time, String name) {
        // Verifica se já existe uma ordem com o mesmo nome
        for (Order order : orders) {
            if (order.getName().equals(name)) {
                telemetry.addLine("Já existe uma ordem com o nome '" + name + "'. Não foi adicionada.");
                return; // Sai do método sem adicionar a nova ordem
            }
        }
        // Se não houver duplicatas, adiciona a nova ordem
        orders.add(new Order(action, time, name));
    }

    public void removeOrderByName(String name) {
        // Itera sobre a lista para encontrar o objeto com o nome correspondente
        for (int i = 0; i < orders.size(); i++) {
            if (orders.get(i).getName().equals(name)) {
                orders.remove(i); // Remove o objeto da lista
                return; // Sai da função após encontrar e remover o item
            }
        }
        telemetry.addLine("Order com o nome '" + name + "' não encontrada.");
    }

    public void checkIfCanRun(double runtime) {
        // Cria uma lista temporária para armazenar ordens a serem removidas
        List<Order> ordersToRemove = new ArrayList<>();

        // Itera sobre a lista de ordens
        for (Order order : orders) {
            // Verifica se o tempo da ordem é menor ou igual ao runtime
            if (order.getTime() <= runtime) {
                runningActions.add(order.getAction()); // Adiciona a ação na lista de runningActions
                ordersToRemove.add(order); // Marca a ordem para remoção

            }
        }

        // Remove as ordens processadas da lista original
        orders.removeAll(ordersToRemove);
    }

    public void runTeleopActions(double runtime) {
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

    }
}

