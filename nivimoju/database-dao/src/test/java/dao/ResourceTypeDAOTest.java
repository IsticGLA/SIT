package dao;

import entity.ResourceType;
import org.junit.AfterClass;
import org.junit.Assert;
import org.junit.BeforeClass;
import org.junit.Test;

/**
 * Created by vivien on 09/04/15.
 */
public class ResourceTypeDAOTest {
    private static ResourceType resourceType;
    private static ResourceTypeDAO resourceTypeDAO;

    @BeforeClass
    public static void init() {
        resourceType = new ResourceType("test_incident");
        resourceTypeDAO = new ResourceTypeDAO();
        DAOManager.connectTest();
    }

    @AfterClass
    public static void close() {
        resourceTypeDAO.disconnect();
    }

    @Test
    public void createTest() {
        ResourceType originalResourceType = resourceTypeDAO.cloneEntity(resourceType);
        ResourceType res = resourceTypeDAO.create(resourceType);

        originalResourceType.setId(res.getId());
        Assert.assertEquals(originalResourceType, res);

    }

    @Test
    public void updateTest(){
        ResourceType res = resourceTypeDAO.create(resourceType);
        res.setLabel("test_update");
        ResourceType updateResourceType = resourceTypeDAO.update(res);
        Assert.assertEquals("test_update", updateResourceType.getLabel());
    }

    @Test
    public void deleteTest() throws InterruptedException {
        ResourceType insertResourceType = resourceTypeDAO.create(resourceType);
        long id = resourceTypeDAO.delete(insertResourceType);
        Assert.assertEquals(insertResourceType.getId(), id);
        ResourceType nullResourceType = resourceTypeDAO.getById(insertResourceType.getId());
        Assert.assertNull(nullResourceType);
    }

    @Test
    public void getByIdTest(){
        ResourceType nullResourceType = resourceTypeDAO.getById(resourceType.getId());
        Assert.assertNull(nullResourceType);

        UserDAO uDAO = new UserDAO();

        ResourceType insertResourceType = resourceTypeDAO.create(resourceType);
        ResourceType getByIdResourceType = resourceTypeDAO.getById(insertResourceType.getId());
        Assert.assertEquals(insertResourceType, getByIdResourceType);
    }
}
