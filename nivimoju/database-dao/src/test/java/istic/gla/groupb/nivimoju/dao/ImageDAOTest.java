package istic.gla.groupb.nivimoju.dao;

import istic.gla.groupb.nivimoju.customObjects.TimestampedPosition;
import istic.gla.groupb.nivimoju.entity.Image;
import istic.gla.groupb.nivimoju.entity.Position;

import org.junit.*;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by jeremy on 19/05/15.
 */
public class ImageDAOTest {
    private static Image iData;
    private static Image iDataTemp;
    private static ImageDAO iDAO;
    private static long timestamp;
    private static double[] position;

    @BeforeClass
    public static void init(){
        iDAO = new ImageDAO();
        iDAO.connectTest();
        position = new double[]{48.5323354, -1.6766565};
        timestamp = 12432432;
        iData = new Image(timestamp, position, "test", 1);
    }

    @AfterClass
    public static void close(){
        List<Image> list = iDAO.getAll();
        for (Image img : list){
            iDAO.delete(img);
        }
        iDAO.disconnect();
    }

    @Before
    public void instantiate(){
        iData = iDAO.create(iData);
        iDataTemp = new Image(timestamp, position, "valueString", 1);
    }

    @After
    public void delete(){
        iDAO.delete(iData);
    }

    @Test
    public void createTest(){
        // Save the data to insert
        Image originalImage = iDAO.cloneEntity(iDataTemp);
        // Insert the data in database
        Image insertedImage = iDAO.create(iDataTemp);
        // Update the id of the original data to match the inserted data
        originalImage.setId(insertedImage.getId());
        // Assert the equality of datas
        org.junit.Assert.assertEquals(originalImage, insertedImage);
        // clean the database
        iDAO.delete(originalImage);
    }

    @Test
    public void getByIdTest(){
        Image res = iDAO.getById(-1l);
        org.junit.Assert.assertNull(res);

        res = iDAO.getById(iData.getId());
        org.junit.Assert.assertEquals(res, iData);
    }

    @Test
    public void getByTest(){
        List<Image> res = iDAO.getBy("timestamp", iData.getTimestamp());
        org.junit.Assert.assertEquals(iData.getTimestamp(), res.get(0).getTimestamp());

        res  = iDAO.getBy("base64Image", iData.getBase64Image());
        org.junit.Assert.assertEquals(iData.getBase64Image(), res.get(0).getBase64Image());
    }

    @Test
    public void updateTest(){
        String base64 = "updateImage";

        // Update an unexisting data
        iDataTemp.setBase64Image(base64);
        Image unexistData = iDAO.update(iDataTemp);
        org.junit.Assert.assertNull(unexistData);

        // Update existing data
        iData.setBase64Image(base64);
        Image updateImage = iDAO.update(iData);
        org.junit.Assert.assertEquals(base64, updateImage.getBase64Image());
    }

    @Test
    public void getAllTest() throws InterruptedException {

        // insert three other data
        iDAO.create(iData);
        iDAO.create(iData);
        iDAO.create(iData);

        // check that we have 4 data in database and clean the database
        int counter = 0;
        List<Image> list = iDAO.getAll();

        for (Image st : list){
            counter++;
            iDAO.delete(st);
        }
        org.junit.Assert.assertEquals(4, counter);
    }

    @Test
    public void deleteTest(){
        long id = iDAO.delete(iDataTemp);
        org.junit.Assert.assertEquals(-1, id);

        id = iDAO.delete(iData);
        org.junit.Assert.assertEquals(id, iData.getId());

        Image nullImage = iDAO.getById(iData.getId());
        org.junit.Assert.assertNull(nullImage);
    }

    @Test
    public void addImageTest() {
        // clean the database
        deleteAll();

        // insert other data
        for (int i = 0; i <= 12; i++){
            iDAO.addImage(iData);
        }

        List<Image> list = iDAO.getAll();
        int counter = list.size();


        for (Image img : list){
            iData.setId(img.getId());
            org.junit.Assert.assertEquals(iData, img);
            iDAO.delete(iData);
        }

        // check that we have 10 images or more
        org.junit.Assert.assertTrue(10 <= counter);
    }

    @Test
    public void getAllLastSpatialImagesTest(){
        // insert other data
        for (int i = 0; i <= 10; i++){
            iData.setTimestamp(iData.getTimestamp() + 1l);
            iDAO.addImage(iData);
        }

        List<TimestampedPosition> list = new ArrayList<>();
        list.add(new TimestampedPosition(new Position(position[0], position[1]), timestamp));
        list.add(new TimestampedPosition(new Position(position[0], position[1]), timestamp + 1l));
        List<Image> lastImages = iDAO.getAllLastSpatialImages(1l, timestamp, 5, list);

        Assert.assertTrue(lastImages.size() >= 5);
        for (Image img : lastImages){
            Assert.assertEquals(img.getPosition()[0], position[0], 0.0000);
            Assert.assertEquals(img.getPosition()[1], position[1], 0.0000);
            Assert.assertNotEquals(img.getTimestamp(), timestamp);
            Assert.assertNotEquals(img.getTimestamp(), timestamp + 1l);
        }
        deleteAll();
    }

    @Test
    public void getLastSpatialImagesTest(){
        // clean database
        deleteAll();

        // insert other data
        for (int i = 0; i <= 10; i++){
            iData.setTimestamp(iData.getTimestamp() + 1l);
            iDAO.addImage(iData);
        }
        List<Image> list = iDAO.getLastSpatialImages(1l, position, timestamp, 5, false);
        Assert.assertTrue(5 >= list.size());
        for (Image img : list){
            Assert.assertEquals(position[0], img.getPosition()[0], 0.000);
            Assert.assertEquals(position[1], img.getPosition()[1], 0.000);
            Assert.assertTrue(img.getTimestamp() >= timestamp);
            Assert.assertEquals(1l, img.getIdIntervention());
        }
        deleteAll();

    }

    @Test
    public void deleteAll(){
        List<Image> list = iDAO.getAll();
        for (Image img : list){
            iDAO.delete(img);
        }
    }
}
