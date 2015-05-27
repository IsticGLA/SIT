package istic.gla.groupb.nivimoju.dao;

import istic.gla.groupb.nivimoju.entity.User;
import istic.gla.groupb.nivimoju.util.ResourceCategory;
import org.junit.*;

import java.util.List;

/**
 * Created by jeremy on 26/05/15.
 */
public class UserDAOTest {
    
    private static User uData;
    private static User uDataTemp;
    private static UserDAO uDAO;

    @BeforeClass
    public static void init(){
        uDAO = new UserDAO();
        uDAO.connectTest();
        uData = new User("login", "password");
        uData = uDAO.create(uData);
    }

    @AfterClass
    public static void close(){
        uDAO.delete(uData);
        uDAO.disconnect();
    }

    @Before
    public void instantiate(){
        uDataTemp = new User("root", "root");
    }

    @Test
    public void connect(){
        uDAO.disconnect();
        uDAO.connect();
        List<User> list = uDAO.getAll();
        Assert.assertTrue(0 < list.size());
        uDAO.disconnect();
        uDAO.connectTest();
    }

    @Test
    public void createTest(){
        // Save the data to insert
        User originalUser = uDAO.cloneEntity(uDataTemp);
        // Insert the data in database
        User insertedUser = uDAO.create(uDataTemp);
        // Update the id of the original data to match the inserted data
        originalUser.setId(insertedUser.getId());
        // Assert the equality of datas
        Assert.assertEquals(originalUser, insertedUser);
        // clean the database
        uDAO.delete(originalUser);
    }

    @Test
    public void getByIdTest(){
        User res = uDAO.getById(-1l);
        Assert.assertNull(res);

        res = uDAO.getById(uData.getId());
        Assert.assertEquals(res, uData);
    }

    @Test
    public void updateTest(){
        String stringTest = "testUser";

        // Update an unexisting data
        uDataTemp.setPassword(stringTest);
        User unexistData = uDAO.update(uDataTemp);
        Assert.assertNull(unexistData);

        // Update existing data
        uData.setPassword(stringTest);
        User updateUser = uDAO.update(uData);
        Assert.assertEquals(stringTest, updateUser.getPassword());
    }

    @Test
    public void getAllTest() throws InterruptedException {
        // insert three other data
        uDAO.create(uData);
        uDAO.create(uData);
        uDAO.create(uData);

        // check that we have 4 data in database and clean the database
        int counter = 0;
        List<User> list = uDAO.getAll();

        for (User st : list){
            counter++;
            uDAO.delete(st);
        }
        Assert.assertEquals(4, counter);
    }

    @Test
    public void deleteTest(){
        long id = uDAO.delete(uDataTemp);
        Assert.assertEquals(-1, id);
        uData = uDAO.create(uData);
        id = uDAO.delete(uData);
        Assert.assertEquals(id, uData.getId());
        User nullUser = uDAO.getById(uData.getId());
        Assert.assertNull(nullUser);
    }
}
