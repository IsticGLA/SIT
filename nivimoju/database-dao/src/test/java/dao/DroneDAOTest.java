package dao;

import entity.Drone;
import org.junit.AfterClass;
import org.junit.Assert;
import org.junit.BeforeClass;
import org.junit.Test;

import java.util.List;

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

    @Test
    public void getByIdIntervention() {
        List<Drone> list = droneDAO.getBy("idIntervention", 10);
        System.out.println(list);
        for (Drone d : list){
            System.out.println(d.getLabel());
        }

    }
}
