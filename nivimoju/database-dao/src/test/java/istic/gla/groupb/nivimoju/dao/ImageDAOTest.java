package istic.gla.groupb.nivimoju.dao;

import istic.gla.groupb.nivimoju.customObjects.TimestampedPosition;
import istic.gla.groupb.nivimoju.entity.Image;
import istic.gla.groupb.nivimoju.entity.Position;
import junit.framework.Assert;
import org.apache.log4j.Logger;
import org.joda.time.DateTime;
import org.junit.AfterClass;
import org.junit.BeforeClass;
import org.junit.Test;

import javax.imageio.ImageIO;
import java.awt.image.BufferedImage;
import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.sql.Timestamp;
import java.util.Calendar;
import java.util.Date;
import java.util.List;

/**
 * Created by jeremy on 19/05/15.
 */
public class ImageDAOTest {
    Logger logger = Logger.getLogger(ImageDAOTest.class);
    private static Image image;
    private static ImageDAO imageDAO;

    @BeforeClass
    public static void init() {
        File fnew = new File("src/test/resources/xray-biker.jpg");
        BufferedImage originalImage= null;
        ByteArrayOutputStream baos = null;
        try {
            originalImage = ImageIO.read(fnew);
            baos = new ByteArrayOutputStream();
            ImageIO.write(originalImage, "jpg", baos);
        } catch (IOException e) {
            e.printStackTrace();
        }
        //byte[] b = baos.toByteArray();
        Date date= new java.util.Date();
        double[] position = {49.73004, -1.46334};
        image = new Image(DateTime.now().getMillis(), position, "testimage", 1);
        imageDAO = new ImageDAO();
        imageDAO.connect();
    }

    @AfterClass
    public static void close() {
        imageDAO.disconnect();
    }

    @Test
    public void createTest() {
        Image res = imageDAO.create(image);
    }

    @Test
    public void getTest() {
        Image res1 = imageDAO.create(image);
        Image res = imageDAO.getById(res1.getId());
        logger.error(res.toString());
        logger.error(res.getType());
        try {
            FileOutputStream fos = new FileOutputStream("src/test/resources/test.jpg");
            //fos.write(res.getBase64Image());
            fos.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    @Test
    public void deleteAll() {
        ImageDAO imgDAO = new ImageDAO();
        List<Image> images = imgDAO.getAll();

        for (Image i : images) {
            imgDAO.delete(i);
        }
    }

    @Test
    public void getSpatialBound(){
        imageDAO.addImage(image);
    }

    @Test
    public void getSpatialTest() {
        Image img = imageDAO.getById(1100l);

        List<TimestampedPosition> test

        List<Image> res = imageDAO.getAllLastSpatialImages(img.getIdIntervention(), img.getTimestamp(), 10, null);
        logger.info("SIZE of the LIST IMAGE : " + res.size());
        for (Image i : res){
            logger.info(i.getBase64Image() + "                   " + i.getPosition()[0] + "   " + i.getPosition()[1]);
        }

        img.setId(-1);
        img.setTimestamp(Calendar.getInstance().getTime().getTime());
        imageDAO.addImage(img);

        img.setId(1100l);
        img.setTimestamp(1432288824313l);
        res = imageDAO.getLastSpatialImages(img.getIdIntervention(), img.getPosition(), img.getTimestamp(), 20);
    }

    @Test
    public void addImage(){
        Image i = imageDAO.addImage(image);
        Assert.assertNotNull(i);
    }
}
