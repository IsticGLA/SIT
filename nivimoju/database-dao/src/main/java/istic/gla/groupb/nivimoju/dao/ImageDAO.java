package istic.gla.groupb.nivimoju.dao;

import com.couchbase.client.java.document.json.JsonArray;
import com.couchbase.client.java.document.json.JsonObject;
import com.couchbase.client.java.error.DocumentDoesNotExistException;
import com.couchbase.client.java.query.Query;
import com.couchbase.client.java.view.*;
import istic.gla.groupb.nivimoju.customObjects.TimestampedPosition;
import istic.gla.groupb.nivimoju.entity.Drone;
import istic.gla.groupb.nivimoju.entity.Image;
import istic.gla.groupb.nivimoju.entity.Intervention;
import istic.gla.groupb.nivimoju.entity.Position;
import istic.gla.groupb.nivimoju.util.Constant;
import org.apache.log4j.Logger;
import org.codehaus.jettison.json.JSONArray;

import java.sql.Timestamp;
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
        logger.debug("creating image in db for position " + img.getPosition()[0] + ";" + img.getPosition()[1] + " and timestamp " +img.getTimestamp());
        cleanImageByPosition(img);
        return create(img);
    }

    public void cleanImageByPosition(Image img) {
        createSpatialLastView();

        JsonArray startKeys = JsonArray.from(img.getIdIntervention(), "" + img.getPosition()[0], "" + img.getPosition()[1]);
        JsonArray endKeys = JsonArray.from(img.getIdIntervention(), "" + img.getPosition()[0], "" + img.getPosition()[1], "");

        List<ViewRow> result = DAOManager.getCurrentBucket().query(ViewQuery.from("designDoc", "single_last_Image").startKey(endKeys).endKey(startKeys).inclusiveEnd(true).descending(true).skip(9)).allRows();

        for(int i=0;i < result.size();i++) {
            try {
                DAOManager.getCurrentBucket().remove(result.get(i).id());
            } catch (DocumentDoesNotExistException e){
                logger.debug("L'objet n'existe pas :", e);
            }
        }
    }


    public final List<Image> getAllLastSpatialImages(Long idIntervention, Long timestamp, int nbImage, List<TimestampedPosition> positionList){
        createSpatialAllLastView();
        JsonArray startKeys = JsonArray.from(idIntervention, timestamp);
        JsonArray endKeys = JsonArray.from(idIntervention, "");

        // set a higher limit to get all interesting points
        int limit = nbImage * 2;

        List<ViewRow> result = DAOManager.getCurrentBucket().query(ViewQuery.from("designDoc", "all_last_Image").startKey(endKeys).endKey(startKeys).inclusiveEnd(true).descending(true).limit(limit)).allRows();

        JsonObject jsonObject = null;
        TimestampedPosition tmpPos = null;
        JsonArray positionDB = null;
        Long timestampDB = null;

        if (positionList != null) {
            for(int i = 0; i < positionList.size();i++) {
                tmpPos = positionList.get(i);
                for(int j = 0; j < result.size(); j++) {
                    jsonObject = (JsonObject) result.get(j).value();
                    positionDB = (JsonArray) jsonObject.get("position");
                    String timestampDBTemp = "" + jsonObject.get("timestamp");
                    timestampDB = Long.valueOf(timestampDBTemp);
                    if(tmpPos.getPosition().getLatitude() == (double) positionDB.get(0) &&
                        tmpPos.getPosition().getLongitude() == (double) positionDB.get(1) &&
                        timestampDB <= tmpPos.getTimestamp()) {
                        result.remove(result.get(j));
                    }
                }
            }
        }

        return viewRowsToEntities(result);
    }

    /**
     * retourne les dernières images pour une position
     * @param idIntervention
     * @param timestamp
     * @param nbImage
     * @param inclusive
     * @return
     */
    public final List<Image> getLastSpatialImages(Long idIntervention, double[] position, Long timestamp, int nbImage, boolean inclusive){
        createSpatialLastView();

        JsonArray startKeys = JsonArray.from(idIntervention, "" + position[0], "" + position[1], timestamp);
        JsonArray endKeys = JsonArray.from(idIntervention, "" + position[0], "" + position[1], "");

        List<ViewRow> result = DAOManager.getCurrentBucket().query(ViewQuery.from("designDoc", "single_last_Image").startKey(endKeys).endKey(startKeys).inclusiveEnd(inclusive).descending(true).limit(nbImage)).allRows();

        return viewRowsToEntities(result);
    }

    private void createSpatialAllLastView()
    {
        DesignDocument designDoc = DAOManager.getCurrentBucket().bucketManager().getDesignDocument("designDoc");
        if (null == designDoc){
            designDoc = createDesignDocument();
        }

        String viewName = "all_last_" + type;
        if (!designDoc.toString().contains(viewName)) {
            String mapFunction =
                    "function (doc, meta) {\n" +
                            "  if (doc.position && doc.type && doc.type == 'Image'){\n" +
                            "    emit([doc.idIntervention, doc.timestamp], doc);\n" +
                            "  }\n" +
                            "}";
            designDoc.views().add(DefaultView.create(viewName, mapFunction));
            DAOManager.getCurrentBucket().bucketManager().upsertDesignDocument(designDoc);
        }
    }

    private void createSpatialLastView()
    {
        DesignDocument designDoc = DAOManager.getCurrentBucket().bucketManager().getDesignDocument("designDoc");
        if (null == designDoc){
            designDoc = createDesignDocument();
        }

        String viewName = "single_last_" + type;
        if (!designDoc.toString().contains(viewName)) {
            String mapFunction =
                    "function (doc, meta) {\n" +
                            "  if (doc.position && doc.type && doc.type == 'Image'){\n" +
                            "       emit([doc.idIntervention, \"\" + doc.position[0], \"\" + doc.position[1], doc.timestamp], doc);\n" +
                            "  }\n" +
                            "}";
            designDoc.views().add(DefaultView.create(viewName, mapFunction));
            DAOManager.getCurrentBucket().bucketManager().upsertDesignDocument(designDoc);
        }
    }
}
