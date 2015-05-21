package istic.gla.groupb.nivimoju.dao;

import com.couchbase.client.java.document.json.JsonArray;
import com.couchbase.client.java.document.json.JsonObject;
import com.couchbase.client.java.query.Query;
import com.couchbase.client.java.view.*;
import istic.gla.groupb.nivimoju.entity.Drone;
import istic.gla.groupb.nivimoju.entity.Image;
import istic.gla.groupb.nivimoju.entity.Intervention;
import istic.gla.groupb.nivimoju.entity.Position;
import istic.gla.groupb.nivimoju.util.Constant;
import org.apache.log4j.Logger;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

/**
 * Created by jeremy on 19/05/15.
 */
public class ImageDAO extends AbstractDAO<Image> {
    Logger logger = Logger.getLogger(ImageDAO.class);

    public ImageDAO() {
        this.typeClass = Image.class;
        this.type = Constant.TYPE_IMAGE;
    }

    public Image addImage(Image img) {
        Position[] tab = img.boundAroundPoint();
        List<Image> listImage = this.getSpatialImages(img.getIdIntervention(), tab[0], tab[1]);
        int size = listImage.size();
        logger.debug("Taille de la liste : " + size);
        long idDelete = -1;
        if (size >= 10) {
            idDelete = this.delete(listImage.get(size - 1));
            if (idDelete != -1) {
                return this.create(img);
            } else {
                return null;
            }
        } else {
            return this.create(img);
        }
    }

    public final List<Image> getSpatialImages(Long idIntervention, Position southern_west, Position northern_east){
        logger.debug(southern_west.getLatitude() + "   " + southern_west.getLongitude() + "              " + northern_east.getLatitude() + "    " + northern_east.getLongitude());
        createSpatialView();
        JsonArray first = JsonArray.from(southern_west.getLatitude(), southern_west.getLongitude());
        JsonArray last = JsonArray.from(northern_east.getLatitude(), northern_east.getLongitude());

        List<SpatialViewRow> result = new ArrayList<>();
        Iterator<SpatialViewRow> iterator = DAOManager.getCurrentBucket().query(SpatialViewQuery.from("designDoc", "spatial_" + type).stale(Stale.FALSE).startRange(first).endRange(last)).iterator();

        while(iterator.hasNext()) {
            SpatialViewRow row = iterator.next();

            JsonObject jsonObject = (JsonObject) row.value();
            if(jsonObject.getLong("idIntervention") == idIntervention) {
                result.add(row);
            }
        }

        return viewSpatialRowsToEntities(result);
    }

    private void createSpatialView()
    {
        DesignDocument designDoc = DAOManager.getCurrentBucket().bucketManager().getDesignDocument("designDoc");
        if (null == designDoc){
            designDoc = createDesignDocument();
        }

        String viewName = "spatial_" + type;
        if (!designDoc.toString().contains(viewName)) {
            String mapFunction =
                    "function(doc, meta)\n" +
                    "{\n" +
                    "  if (doc.position && doc.type && doc.type == '" + type + "')\n" +
                    "  {\n" +
                    "     emit(\n" +
                    "          {\n" +
                    "             type: \"Point\",\n" +
                    "             coordinates: doc.position,\n" +
                    "          }, doc);\n" +
                    "  }\n" +
                    "}";
            designDoc.views().add(SpatialView.create(viewName, mapFunction));
            DAOManager.getCurrentBucket().bucketManager().upsertDesignDocument(designDoc);
        }
    }
}
