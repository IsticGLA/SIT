package istic.gla.groupeb.flerjeco.springRest;

import android.util.Log;

import org.springframework.http.HttpMethod;
import org.springframework.http.ResponseEntity;
import org.springframework.http.converter.json.MappingJackson2HttpMessageConverter;
import org.springframework.web.client.HttpStatusCodeException;
import org.springframework.web.client.RestTemplate;

import java.util.Arrays;
import java.util.List;

import entity.IncidentCode;
import entity.Intervention;
import entity.ResourceType;
import entity.StaticData;

/**
 * Created by amhachi on 08/04/15.
 */
public class SpringService {
    private static final String TAG = SpringService.class.getSimpleName();
    private static final String URL = "http://ns3002211.ip-37-59-58.eu:8080/nivimoju/rest/";

    boolean test = true;

    /**
     * get resource by id
     * @param idRes id of the resource to get
     * @return the resource type retrieved
     */
    public ResourceType getResourceTypeById(Long idRes){

        final String url = URL + "resource/"+idRes;

        RestTemplate restTemplate = new RestTemplate();

        MappingJackson2HttpMessageConverter mappingJackson2HttpMessageConverter = new MappingJackson2HttpMessageConverter();

        restTemplate.getMessageConverters().add(mappingJackson2HttpMessageConverter);

        ResponseEntity<ResourceType> resourcetype = restTemplate.getForEntity(url, ResourceType.class);

        ResourceType rt = resourcetype.getBody();
        return rt;
    }

    /**
     * Get incident codes
     * @return array of IncidentCode
     * @throws HttpStatusCodeException throw exception if status code is bad
     */
    public IncidentCode[] codeSinistreClient() throws HttpStatusCodeException {

        final String url = URL + "incidentcode";

        RestTemplate restTemplate = new RestTemplate();

        MappingJackson2HttpMessageConverter mappingJackson2HttpMessageConverter = new MappingJackson2HttpMessageConverter();

        restTemplate.getMessageConverters().add(mappingJackson2HttpMessageConverter);

        ResponseEntity<IncidentCode[]> incidentCode = restTemplate.getForEntity(url, IncidentCode[].class);

        IncidentCode[] codes = incidentCode.getBody();
        return codes;


    }

    /**
     * Create intervention
     * @param intervention intervention to be created
     * @return id of the intervention created
     */
    public Intervention postIntervention(Intervention intervention) {
        try {

            intervention.updateDate();

            final String url = URL + "intervention/create";

            RestTemplate restTemplate = new RestTemplate();
            restTemplate.getMessageConverters().add(new MappingJackson2HttpMessageConverter());

            ResponseEntity<Intervention> intervetionResult = restTemplate.postForEntity(url, intervention, Intervention.class);

            if (intervetionResult == null) {
                Log.i("MAMH", "intervetionResult = null");
            } else
                return intervetionResult.getBody();
        } catch (HttpStatusCodeException e) {
            Log.i("MAMH", "Problème de la création de l'intervention : " + e.getMessage());
        }
        return null;
    }

    public Intervention updateIntervention(Intervention intervention) {
        try {
            intervention.updateDate();
            final String url = URL + "intervention/update";

            RestTemplate restTemplate = new RestTemplate();
            restTemplate.getMessageConverters().add(new MappingJackson2HttpMessageConverter());

            ResponseEntity<Intervention> intervetionResult = restTemplate.postForEntity(url, intervention, Intervention.class);

            if (intervetionResult == null) {
                Log.i("MAMH", "intervetionResult = null");
            } else
                Log.i(TAG, intervetionResult.toString());
                return intervetionResult.getBody();
        } catch (HttpStatusCodeException e) {
            Log.i("MAMH", "Problème de l'update de l'intervention : " + e.getMessage());
        }
        return null;
    }

    /**
     *
     * @param id
     * @param password
     * @return
     */
    public String login(String id, String password) {
        Log.i(TAG, "login start");
        final String url = URL + "authentication/connected/" + id + "/" + password;
        String httpResult = "";

        RestTemplate restTemplate = new RestTemplate();
        restTemplate.getMessageConverters().add(new MappingJackson2HttpMessageConverter());

        try {
            ResponseEntity<String> entity = restTemplate.getForEntity(url, String.class);
            httpResult = entity.getStatusCode().toString();
        } catch (HttpStatusCodeException e) {
            httpResult = "400";
        }
        Log.i(TAG, "httpResult : " + httpResult);
        Log.i(TAG, "login end");
        return httpResult;
    }

    /**
     * get a notification from server
     * @return intervention to update
     */
    public Intervention getNotify() {
        Log.i(TAG, "notify start");
        final String url = URL + "notify";


        RestTemplate restTemplate = new RestTemplate();

        MappingJackson2HttpMessageConverter mappingJackson2HttpMessageConverter = new MappingJackson2HttpMessageConverter();

        restTemplate.getMessageConverters().add(mappingJackson2HttpMessageConverter);

        ResponseEntity<Intervention> interventionTest = restTemplate.getForEntity(url, Intervention.class);

        Intervention rt = interventionTest.getBody();
        return rt;
    }


    public ResourceType[] resourceTypes() {
        final String url = URL + "resource";

        RestTemplate restTemplate = new RestTemplate();
        restTemplate.getMessageConverters().add(new MappingJackson2HttpMessageConverter());

        ResponseEntity<ResourceType[]> resourceTypes = restTemplate.getForEntity(url, ResourceType[].class);
        return resourceTypes.getBody();
    }

    public Long requestVehicle(Object[] params) {
        final String url = URL + "intervention/" + params[0] + "/resources/" + params[1];
        RestTemplate restTemplate = new RestTemplate();
        restTemplate.getMessageConverters().add(new MappingJackson2HttpMessageConverter());

        ResponseEntity<Long> requestId = restTemplate.exchange(url, HttpMethod.PUT, null, Long.class);
        return requestId.getBody();
    }

    public Intervention[] getAllInterventions() {
        Log.i(TAG, "getAllInterventions start");
        final String url = URL + "intervention";
        RestTemplate restTemplate = new RestTemplate();
        restTemplate.getMessageConverters().add(new );

        ResponseEntity<Intervention[]> entity = restTemplate.getForEntity(url, Intervention[].class);
        Log.i(TAG, "getAllInterventions : " + entity.getBody().toString());
        return entity.getBody();
    }

    public Long moveDrone(Object[] params) {
        Log.i(TAG, "move drone to : " + params[0] + ", " + params[1]);
        final String url = URL + "drone/move/" + params[0] + "/" + params[1];
        RestTemplate restTemplate = new RestTemplate();
        restTemplate.getMessageConverters().add(new MappingJackson2HttpMessageConverter());

        ResponseEntity<Long> id = restTemplate.exchange(url, HttpMethod.GET, null, Long.class);
        return id.getBody();
    }

    public Intervention changeResourceState(Object[] params) {
        final String url = URL + "intervention/" + params[0] + "/resources/" + params[1] + "/" + params[2] + "/" + params[3];
        RestTemplate restTemplate = new RestTemplate();
        restTemplate.getMessageConverters().add(new MappingJackson2HttpMessageConverter());

        ResponseEntity<Intervention> id = restTemplate.exchange(url, HttpMethod.PUT, null, Intervention.class);
        return id.getBody();
    }
    public StaticData[] getAllStaticDatas() {
        Log.i(TAG, "getAllStaticDatas start");
        final String url = URL + "staticdata";
        RestTemplate restTemplate = new RestTemplate();
        restTemplate.getMessageConverters().add(new MappingJackson2HttpMessageConverter());

        ResponseEntity<StaticData[]> entity = restTemplate.getForEntity(url, StaticData[].class);
        Log.i(TAG, "getAllStaticData : " + entity.getBody().toString());
        return entity.getBody();
    }
}
