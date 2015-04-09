package dao;

import com.couchbase.client.deps.com.fasterxml.jackson.core.JsonProcessingException;
import com.couchbase.client.deps.com.fasterxml.jackson.databind.ObjectMapper;
import com.couchbase.client.deps.com.fasterxml.jackson.databind.ObjectWriter;
import com.couchbase.client.java.document.JsonDocument;
import com.couchbase.client.java.document.json.JsonObject;
import entity.IncidentCode;
import util.Constant;

import java.io.IOException;

/**
 * Created by jeremy on 09/04/15.
 */
public class IncidentCodeDAO extends AbstractDAO<IncidentCode> {

    public IncidentCodeDAO() {
        this.type = Constant.TYPE_INCIDENT_CODE;
    }

    @Override
    protected IncidentCode jsonDocumentToEntity(JsonDocument jsonDocument) {
        IncidentCode incidentCode = new IncidentCode();

        try {
            ObjectMapper om = new ObjectMapper();
            incidentCode = om.readValue(jsonDocument.content().toString(), IncidentCode.class);
        } catch (IOException e) {
            e.printStackTrace();
        }

        return incidentCode;
    }

    @Override
    protected JsonDocument entityToJsonDocument(IncidentCode entity) {
        String json = "";
        try {
            ObjectWriter ow = new ObjectMapper().writer().withDefaultPrettyPrinter();
            json = ow.writeValueAsString(entity);
        } catch (JsonProcessingException e) {
            e.printStackTrace();
        }
        return JsonDocument.create(Long.toString(entity.getId()), JsonObject.fromJson(json));
    }
}
