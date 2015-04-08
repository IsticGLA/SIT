package dao;


import entity.Position;
import org.junit.*;
import util.Configuration;

import java.util.HashMap;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNull;

/**
 * Created by arno on 17/03/15.
 */
public class GeoDroneDAOTest {
    public void testGetAll()
    {
        //dao.createDesignDocument();
        for(GeoDrone g: dao.getAll()){
            System.out.println(g);
        }
    }


    private static GeoDroneDAO dao = new GeoDroneDAO();
    private static byte[] imagesBytes = new byte[16];
    @BeforeClass
    public static void beforeAllTests() {
        HashMap<String, String> configs = new HashMap<String, String>();
        configs.put("COUCHBASE_HOSTNAME","37.59.58.42");
        configs.put("BUCKET_NAME","test");
        Configuration.loadConfigurations(configs);
        dao.connect();
        for(int i = 0 ; i<imagesBytes.length; i = i +1)
        {
            imagesBytes[i] = 1;
        }
    }

    @AfterClass
    public static void afterAllTests() {
        //CouchbaseCluster.create(Configuration.COUCHBASE_HOSTNAME).openBucket("e").;
        dao.disconnect();
    }

    @Before
    public void setUp() throws Exception {

    }

    @After
    public void tearDown() {


    }

    @Test
    public void test()
    {
        GeoDrone geoDrone = new GeoDrone();
        geoDrone.setCoordinates(new Position(4.0, 9.0, 19.0));
        dao.entityToJsonDocument(geoDrone);
    }

    @Test
    public void testInsert() {
        GeoDrone geoDrone = new GeoDrone();
        geoDrone.setCoordinates(new Position(4.0, 9.0, 19.0));
        dao.entityToJsonDocument(geoDrone);

        GeoDrone res = dao.create(geoDrone);
        assertEquals(geoDrone, res);
        assertEquals(geoDrone.getId(), res.getId());
    }

    @Test
    public void testUpdate() {

        //insertion
        GeoDrone geoDrone = new GeoDrone();
        geoDrone.setCoordinates(new Position(4.0, 9.0, 19.0));
        dao.entityToJsonDocument(geoDrone);

        GeoDrone res = dao.create(geoDrone);
        long idInbase = res.getId();
        assertEquals(geoDrone, res);
        assertEquals(geoDrone.getId(), res.getId());

        // update
        geoDrone = dao.getById(idInbase);
        geoDrone.setCoordinates(new Position(3.0, 2.0, 1.2));
        res = dao.update(geoDrone);
        assertEquals(geoDrone, res);
        assertEquals(geoDrone.getId(), res.getId());
    }

    @Test
    public void testDelete() {
        //insertion
        GeoDrone geoDrone = new GeoDrone();
        geoDrone.setCoordinates(new Position(4.0, 9.0, 19.0));


        GeoDrone res = dao.create(geoDrone);
        long idInbase = res.getId();
        assertEquals(geoDrone, res);
        assertEquals(geoDrone.getId(), res.getId());

        // suppression
        dao.delete(geoDrone);
        assertNull(dao.getById(idInbase));
    }
}
