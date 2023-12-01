package frc.robot.stateMachine;

import java.util.ArrayList;
import java.util.HashMap;

public class StateMachine {
    private String currentState;
    private String initialState;
    private HashMap<String, ArrayList<Edge>> edges;

    public StateMachine(String init) {
        initialState = init;
        currentState = init;
        edges = new HashMap<>();
        edges.put(init, new ArrayList<Edge>());
    }
    public String getCurrentState() {
        return currentState;
    }
    public void addEdge(Edge edge) {
       if(edges.containsKey(edge.getBaseState())) { // modify existing arraylist
            ArrayList<Edge> currentEdges = edges.get(edge.getBaseState());
            currentEdges.add(edge);
            edges.put(edge.getBaseState(), currentEdges);
       } else { // create new key-value
            ArrayList<Edge> newEdges = new ArrayList<Edge>();
            newEdges.add(edge);
            edges.put(edge.getBaseState(), newEdges);
       }
    }
    public void command(String edge) {
        ArrayList<Edge> edgesFromCurrentState = edges.get(currentState);
        for (Edge e : edgesFromCurrentState) {
            if(e.getEdgeName() == edge) {
                // switch state to next edge
                currentState = e.getEndState();
                break;
            }
            // currentState won't change otherwise
        }
    }
}
