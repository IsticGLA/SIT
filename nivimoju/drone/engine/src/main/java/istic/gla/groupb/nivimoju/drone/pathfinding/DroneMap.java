package istic.gla.groupb.nivimoju.drone.pathfinding;

import istic.gla.groupb.nivimoju.drone.engine.Node;

/**
 * Created by sacapuces on 18/05/15.
 */
public class DroneMap implements TileBasedMap {
    Node[][] map;

    @Override
    public int getWidthInTiles() {
        return map.length;
    }

    @Override
    public int getHeightInTiles() {
        if(map.length == 0)
            return 0;
        return map[0].length;
    }

    @Override
    public void pathFinderVisited(int x, int y) {

    }

    @Override
    public boolean blocked(int x, int y) {
        return map[x][y].isObstacle();
    }

    @Override
    public float getCost(int sx, int sy, int tx, int ty) {
        return 1;
    }
}
