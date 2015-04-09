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
        resourceTypeDAO.connect();
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
}
