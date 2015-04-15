package dao;

import entity.Drone;
import org.junit.AfterClass;
import org.junit.Assert;
import org.junit.BeforeClass;
import org.junit.Test;

/**
 * Created by jeremy on 14/04/15.
 */
public class DroneDAOTest {

    private static Drone drone;
    private static DroneDAO droneDAO;

    @BeforeClass
    public static void init() {
        drone = new Drone("Drone 1");
        droneDAO = new DroneDAO();
        DAOManager.connectTest();
    }

    @AfterClass
    public static void close() {
        droneDAO.disconnect();
    }

    @Test
    public void createTest() {
        Drone originalDrone = droneDAO.cloneEntity(drone);
        Drone res = droneDAO.create(drone);

        originalDrone.setId(res.getId());
        Assert.assertEquals(originalDrone, res);

    }
}
