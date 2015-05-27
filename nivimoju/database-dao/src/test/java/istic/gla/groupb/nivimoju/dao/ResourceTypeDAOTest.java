package istic.gla.groupb.nivimoju.dao;

import istic.gla.groupb.nivimoju.entity.ResourceType;
import istic.gla.groupb.nivimoju.entity.ResourceType;
import istic.gla.groupb.nivimoju.util.MarkerType;
import istic.gla.groupb.nivimoju.util.ResourceCategory;
import org.junit.*;

import java.util.List;

/**
 * Created by vivien on 09/04/15.
 */
public class ResourceTypeDAOTest {

    private static ResourceType rtData;
    private static ResourceType rtDataTemp;
    private static ResourceTypeDAO rtDAO;

    @BeforeClass
    public static void init(){
        rtDAO = new ResourceTypeDAO();
        rtDAO.connectTest();
        rtData = new ResourceType("test", ResourceCategory.vehicule);
        rtData = rtDAO.create(rtData);
    }

    @AfterClass
    public static void close(){
        rtDAO.delete(rtData);
        rtDAO.disconnect();
    }

    @Before
    public void instantiate(){
        rtDataTemp = new ResourceType("test2", ResourceCategory.dragabledata);
    }

    @Test
    public void createTest(){
        // Save the data to insert
        ResourceType originalResourceType = rtDAO.cloneEntity(rtDataTemp);
        // Insert the data in database
        ResourceType insertedResourceType = rtDAO.create(rtDataTemp);
        // Update the id of the original data to match the inserted data
        originalResourceType.setId(insertedResourceType.getId());
        // Assert the equality of datas
        Assert.assertEquals(originalResourceType, insertedResourceType);
        // clean the database
        rtDAO.delete(originalResourceType);
    }

    @Test
    public void getByIdTest(){
        ResourceType res = rtDAO.getById(-1l);
        Assert.assertNull(res);

        res = rtDAO.getById(rtData.getId());
        Assert.assertEquals(res, rtData);
    }

    @Test
    public void updateTest(){
        String stringTest = "coucou";

        // Update an unexisting data
        rtDataTemp.setLabel(stringTest);
        ResourceType unexistData = rtDAO.update(rtDataTemp);
        Assert.assertNull(unexistData);

        // Update existing data
        rtData.setLabel(stringTest);
        ResourceType updateResourceType = rtDAO.update(rtData);
        Assert.assertEquals(stringTest, updateResourceType.getLabel());
    }

    @Test
    public void getAllTest() throws InterruptedException {
        // insert three other data
        rtDAO.create(rtData);
        rtDAO.create(rtData);
        rtDAO.create(rtData);

        // check that we have 4 data in database and clean the database
        int counter = 0;
        List<ResourceType> list = rtDAO.getAll();

        for (ResourceType st : list){
            counter++;
            rtDAO.delete(st);
        }
        Assert.assertEquals(4, counter);
    }

    @Test
    public void deleteTest(){
        long id = rtDAO.delete(rtDataTemp);
        Assert.assertEquals(-1, id);
        rtData = rtDAO.create(rtData);
        id = rtDAO.delete(rtData);
        Assert.assertEquals(id, rtData.getId());
        ResourceType nullResourceType = rtDAO.getById(rtData.getId());
        Assert.assertNull(nullResourceType);
    }
}
