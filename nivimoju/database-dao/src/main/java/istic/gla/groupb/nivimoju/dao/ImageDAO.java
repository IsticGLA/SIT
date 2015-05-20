package istic.gla.groupb.nivimoju.dao;

import com.couchbase.client.java.document.json.JsonArray;
import com.couchbase.client.java.query.Query;
import com.couchbase.client.java.view.*;
import istic.gla.groupb.nivimoju.entity.Drone;
import istic.gla.groupb.nivimoju.entity.Image;
import istic.gla.groupb.nivimoju.entity.Intervention;
import istic.gla.groupb.nivimoju.entity.Position;
import istic.gla.groupb.nivimoju.util.Constant;

import java.util.List;

/**
 * Created by jeremy on 19/05/15.
 */
public class ImageDAO extends AbstractDAO<Image> {

    public ImageDAO() {
        this.typeClass = Image.class;
        this.type = Constant.TYPE_IMAGE;
    }

    public final List<Image> getSpatialImages(Position southern_west, Position northern_east){
        createSpatialView();
        JsonArray first = JsonArray.from(southern_west.getLatitude(), southern_west.getLongitude());
        JsonArray last = JsonArray.from(northern_east.getLatitude(), northern_east.getLongitude());
        List<SpatialViewRow> result = DAOManager.getCurrentBucket().query(SpatialViewQuery.from("designDoc", "spatial_" + type).stale(Stale.FALSE).startRange(first).endRange(last)).allRows();
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
