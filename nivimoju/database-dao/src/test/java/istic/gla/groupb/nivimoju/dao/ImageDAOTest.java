package istic.gla.groupb.nivimoju.dao;

import istic.gla.groupb.nivimoju.entity.Image;
import istic.gla.groupb.nivimoju.entity.Position;
import org.apache.log4j.Logger;
import org.joda.time.DateTime;
import org.junit.AfterClass;
import org.junit.BeforeClass;
import org.junit.Test;

import javax.imageio.ImageIO;
import javax.imageio.stream.ImageOutputStream;
import java.awt.image.BufferedImage;
import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.sql.Timestamp;
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
        byte[] b = baos.toByteArray();
        Date date= new java.util.Date();
        double[] position = {49.63004, -1.66334};
        image = new Image(256.0, DateTime.now().getMillis(), position, b, 1);
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
        Image res = imageDAO.getById(53l);
        logger.error(res.toString());
        logger.error(res.getType());
        try {
            FileOutputStream fos = new FileOutputStream("src/test/resources/test.jpg");
            fos.write(res.getImage());
            fos.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    @Test
    public void getSpatialTest() {
        Position first = new Position(49.6, -1.7);
        Position last = new Position(50.0, -1.6);
        List<Image> res = imageDAO.getSpatialImages(first, last);
        for (Image i : res){
            logger.info(i.getPosition()[0] + "   " + i.getPosition()[1]);
        }
    }
}
