package istic.gla.groupb.nivimoju.dao;

import istic.gla.groupb.nivimoju.entity.Image;
import istic.gla.groupb.nivimoju.entity.Position;
import org.apache.log4j.Logger;
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
        image = new Image(256.0, 256.0, new Timestamp(date.getTime()), new Position(48.63004, -1.66334), b);
        imageDAO = new ImageDAO();
        imageDAO.connect();
    }

    @AfterClass
    public static void close() {
        imageDAO.disconnect();
    }

    @Test
    public void createTest() {
        //Image res = imageDAO.create(image);
    }

    @Test
    public void getTest() {
        Image res = imageDAO.getById(49l);
        try {
            FileOutputStream fos = new FileOutputStream("src/test/resources/test.jpg");
            fos.write(res.getImage());
            fos.close();
        } catch (IOException e) {
            e.printStackTrace();
        }

    }
}
