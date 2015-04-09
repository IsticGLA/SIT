package dao;

import com.couchbase.client.deps.com.fasterxml.jackson.core.JsonProcessingException;
import com.couchbase.client.deps.com.fasterxml.jackson.databind.ObjectMapper;
import com.couchbase.client.deps.com.fasterxml.jackson.databind.ObjectWriter;
import com.couchbase.client.java.document.JsonDocument;
import com.couchbase.client.java.document.json.JsonObject;
import entity.User;
import util.Constant;

import java.io.IOException;

/**
 * Created by vivien on 08/04/15.
 */
public class UserDAO extends AbstractDAO<User> {

    public UserDAO() {
        this.type = Constant.TYPE_USER;
    }

    @Override
    protected User jsonDocumentToEntity(JsonDocument jsonDocument) {
        User user = new User();

        try {
            ObjectMapper om = new ObjectMapper();
            user = om.readValue(jsonDocument.content().toString(), User.class);
        } catch (IOException e) {
            e.printStackTrace();
        }

        return user;
    }

    @Override
    protected JsonDocument entityToJsonDocument(User entity) {
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
