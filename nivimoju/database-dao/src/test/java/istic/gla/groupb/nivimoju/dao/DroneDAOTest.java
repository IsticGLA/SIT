package istic.gla.groupb.nivimoju.dao;

import istic.gla.groupb.nivimoju.entity.Drone;
import istic.gla.groupb.nivimoju.util.MarkerType;
import org.junit.*;

import java.util.List;

/**
 * Created by jeremy on 14/04/15.
 */
public class DroneDAOTest {
    private static Drone dData;
    private static Drone dDataTemp;
    private static DroneDAO dDAO;

    @BeforeClass
    public static void init(){
        dDAO = new DroneDAO();
        dDAO.connectTest();
        dData = new Drone("drone_1", 48.666, -1.64000, 2l);
        dData = dDAO.create(dData);
    }

    @AfterClass
    public static void close(){
        dDAO.delete(dData);
        dDAO.disconnect();
    }

    @Before
    public void instantiate(){
        dDataTemp = new Drone("drone_2", 48.666, -1.64000, 1l);
    }

    @Test
    public void createTest(){
        // Save the data to insert
        Drone originalDrone = dDAO.cloneEntity(dDataTemp);
        // Insert the data in database
        Drone insertedDrone = dDAO.create(dDataTemp);
        // Update the id of the original data to match the inserted data
        originalDrone.setId(insertedDrone.getId());
        // Assert the equality of datas
        Assert.assertEquals(originalDrone, insertedDrone);
        // clean the database
        dDAO.delete(originalDrone);
    }

    @Test
    public void getByIdTest(){
        Drone res = dDAO.getById(-1l);
        Assert.assertNull(res);

        res = dDAO.getById(dData.getId());
        Assert.assertEquals(res, dData);
    }

    @Test
    public void getByTest(){
        dData = dDAO.create(dData);
        List<Drone> res = dDAO.getBy("label", dData.getLabel());
        Assert.assertEquals(dData.getLabel(), res.get(0).getLabel());

        dDAO.delete(dData);

    }

    @Test
    public void updateTest(){
        long newLat = (long) 47.6334;

        // Update an unexisting data
        dDataTemp.setLatitude(newLat);
        Drone unexidData = dDAO.update(dDataTemp);
        Assert.assertNull(unexidData);

        // Update existing data
        dData.setLatitude(newLat);
        Drone updateDrone = dDAO.update(dData);
        Assert.assertEquals(newLat, updateDrone.getLatitude(), 0.0000);
    }

    @Test
    public void getAllTest() throws InterruptedException {
        // insert three other data
        dDAO.create(dData);
        dDAO.create(dData);
        dDAO.create(dData);

        // check that we have 4 data in database and clean the database
        int counter = 0;
        List<Drone> list = dDAO.getAll();

        for (Drone st : list){
            counter++;
            dDAO.delete(st);
        }
        Assert.assertEquals(4, counter);
    }

    @Test
    public void deleteTest(){
        long id = dDAO.delete(dDataTemp);
        Assert.assertEquals(-1, id);
        dData = dDAO.create(dData);
        id = dDAO.delete(dData);
        Assert.assertEquals(id, dData.getId());
        Drone nullDrone = dDAO.getById(dData.getId());
        Assert.assertNull(nullDrone);
    }
}
