package frc.robot.stateMachine;

public class Edge {
    private String baseState;
    private String endState;
    private String edgeName;
    public Edge (String start, String end, String e) {
        baseState = start;
        endState = end;
        edgeName = e;
    }
    public String getBaseState() {
        return baseState;
    }
    public String getEndState() {
        return endState;
    }
    public String getEdgeName() {
        return edgeName;
    }
}
