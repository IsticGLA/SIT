package istic.gla.groupb.nivimoju.drone.engine;

import istic.gla.groupb.nivimoju.entity.Position;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by sacapuces on 17/5/2015.
 */
public class DroneEngineTest {
    @Test
    public void test1(){
        DroneEngine engine = DroneEngine.getInstance();
        List<Position> positions = new ArrayList<>();
        positions.add(new Position(48.115482, -1.637948));
        positions.add(new Position(48.115559, -1.637701));
        positions.add(new Position(48.115386, -1.637491));
        positions.add(new Position(48.115196, -1.637826));
        positions.add(new Position(48.115369,-1.638118));
        positions.add(new Position(48.115607, -1.638259));
        engine.getPathForScan(positions, 3);
    }
}
