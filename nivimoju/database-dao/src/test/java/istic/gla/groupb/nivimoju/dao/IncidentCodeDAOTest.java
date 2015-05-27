package istic.gla.groupb.nivimoju.dao;

import istic.gla.groupb.nivimoju.entity.IncidentCode;
import istic.gla.groupb.nivimoju.util.ResourceCategory;
import org.junit.*;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by jeremy on 26/05/15.
 */
public class IncidentCodeDAOTest {

    private static IncidentCode icData;
    private static IncidentCode icDataTemp;
    private static IncidentCodeDAO icDAO;

    @BeforeClass
    public static void init(){
        icDAO = new IncidentCodeDAO();
        icDAO.connectTest();
        List<Long> list = new ArrayList<>();
        list.add(3l);
        list.add(4l);
        icData = new IncidentCode("Test", list);
        icData = icDAO.create(icData);
    }

    @AfterClass
    public static void close(){
        icDAO.delete(icData);
        icDAO.disconnect();
    }

    @Before
    public void instantiate(){
        List<Long> list = new ArrayList<>();
        list.add(3l);
        list.add(4l);
        icDataTemp = new IncidentCode("Test2", list);
    }

    @Test
    public void createTest(){
        // Save the data to insert
        IncidentCode originalIncidentCode = icDAO.cloneEntity(icDataTemp);
        // Insert the data in database
        IncidentCode insertedIncidentCode = icDAO.create(icDataTemp);
        // Update the id of the original data to match the inserted data
        originalIncidentCode.setId(insertedIncidentCode.getId());
        // Assert the equality of datas
        Assert.assertEquals(originalIncidentCode, insertedIncidentCode);
        // clean the database
        icDAO.delete(originalIncidentCode);
    }

    @Test
    public void getByIdTest(){
        IncidentCode res = icDAO.getById(-1l);
        Assert.assertNull(res);

        res = icDAO.getById(icData.getId());
        Assert.assertEquals(res, icData);
    }

    @Test
    public void updateTest(){
        List<Long> list = new ArrayList<>();
        list.add(3l);
        list.add(4l);

        // Update an unexisting data
        icDataTemp.setresourceType(list);
        IncidentCode unexistData = icDAO.update(icDataTemp);
        Assert.assertNull(unexistData);

        // Update existing data
        icData.setresourceType(list);
        IncidentCode updateIncidentCode = icDAO.update(icData);
        Assert.assertEquals(list, updateIncidentCode.getresourceType());
    }

    @Test
    public void getAllTest() throws InterruptedException {
        // insert three other data
        icDAO.create(icData);
        icDAO.create(icData);
        icDAO.create(icData);

        // check that we have 4 data in database and clean the database
        int counter = 0;
        List<IncidentCode> list = icDAO.getAll();

        for (IncidentCode st : list){
            counter++;
            icDAO.delete(st);
        }
        Assert.assertEquals(4, counter);
    }

    @Test
    public void deleteTest(){
        long id = icDAO.delete(icDataTemp);
        Assert.assertEquals(-1, id);
        icData = icDAO.create(icData);
        id = icDAO.delete(icData);
        Assert.assertEquals(id, icData.getId());
        IncidentCode nullIncidentCode = icDAO.getById(icData.getId());
        Assert.assertNull(nullIncidentCode);
    }
}
