package dao;

import entity.Drone;
import entity.User;
import org.apache.log4j.Logger;
import org.junit.AfterClass;
import org.junit.Assert;
import org.junit.BeforeClass;
import org.junit.Test;

import java.util.List;

/**
 * Created by jeremy on 14/04/15.
 */
public class DroneDAOTest {
    Logger logger = Logger.getLogger(DroneDAOTest.class);
    private static Drone drone;
    private static DroneDAO droneDAO;

    @BeforeClass
    public static void init() {
        drone = new Drone("Drone 1");
        droneDAO = new DroneDAO();
        droneDAO.connect();
    }

    @AfterClass
    public static void close() {
        droneDAO.disconnect();
    }

    @Test
    public void createTest() {
        Drone originalDrone = droneDAO.cloneEntity(drone);
        //Drone res = droneDAO.create(drone);
        String login = "a";
        List<User> u = new UserDAO().getBy("login", login);
        logger.info(u.size());
        logger.info(u.get(0).getLogin() + "  " + u.get(0).getPassword());

        //originalDrone.setId(res.getId());
        //Assert.assertEquals(originalDrone, res);

    }

    @Test
    public void getByIdIntervention() {
        List<Drone> list = droneDAO.getBy("idIntervention", 10);
        logger.info(list);
        for (Drone d : list){
            logger.info(d.getLabel());
        }

    }

    @Test
    public void getAll() {
        List<Drone> list = droneDAO.getAll();
        logger.info(list);
        for (Drone d : list){
            logger.info(d.getLabel());
        }
    }

    @Test
    public void getUnassign() {
        List<Drone> list = droneDAO.getBy("idIntervention", -1l);
        logger.info(list);
        for (Drone d : list){
            logger.info(d.getLabel());
            logger.info(d.getIdIntervention());
        }
    }

    @Test
    public void testAssign() {

        List<entity.Drone> droneList = droneDAO.getBy("idIntervention", -1);
        if (null != droneList && droneList.size() > 1) {
            logger.info(droneList.size());
            entity.Drone updateDrone = droneList.get(0);
            updateDrone.setIdIntervention(46);
            updateDrone.updateDate();
            logger.info(updateDrone.getIdIntervention() + "  " + updateDrone.getId());
            Drone d = droneDAO.update(updateDrone);
            logger.info(d.getIdIntervention() + "  " + d.getId());
            Assert.assertEquals(updateDrone, d);

        }
    }

    @Test
    public void update() {
        drone = droneDAO.getById(25l);
        drone.setIdIntervention(10);
        drone.updateDate();
        Drone updatedDrone = droneDAO.update(drone);
        logger.info(updatedDrone);
        Assert.assertEquals(10, updatedDrone.getIdIntervention());
    }
}
