package istic.gla.goupb.nivimoju.drone.engine;

import istic.gla.groupb.nivimoju.drone.latlong.LocalCoordinate;

/**
 * représente un noeud de parcours pour un scan
 */
public class Node {
    private boolean obstacle;
    private boolean visited;
    private boolean toScan;
    private LocalCoordinate position;

    public Node() {
        obstacle = false;
        visited = false;
        toScan = false;
    }

    public Node(boolean obstacle, boolean visited, boolean toScan) {
        this.obstacle = obstacle;
        this.visited = visited;
        this.toScan = toScan;
    }

    public boolean isObstacle() {
        return obstacle;
    }

    public void setObstacle(boolean obstacle) {
        this.obstacle = obstacle;
    }

    public boolean isVisited() {
        return visited;
    }

    public void setVisited(boolean visited) {
        this.visited = visited;
    }

    public boolean isToScan() {
        return toScan;
    }

    public void setToScan(boolean toScan) {
        this.toScan = toScan;
    }

    public LocalCoordinate getPosition() {
        return position;
    }

    public void setPosition(LocalCoordinate position) {
        this.position = position;
    }
}
