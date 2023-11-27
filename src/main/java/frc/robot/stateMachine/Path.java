package frc.robot.stateMachine;

public class Path {
    private String baseState;
    private String endState;
    private String[] path;
    private int index;
    public Path (String b, String[] p) {
        baseState = b;
        path = p;
        endState = p[-1];
        index = 0;
    }
    public boolean checkForValidPath(String start, String desired) {
        return desired.equals(endState) && start.equals(baseState);
    }
    public String getBaseState() {
        return baseState;
    }
    public String getCurrentState() {
        return path[index];
    }
    public String nextEdge() {
        index++;
        if (index > path.length) {
            /* already at end of path? 
                loop around? 
                or traverse backwards?
            */
        }
        return getCurrentState();
    }
}
