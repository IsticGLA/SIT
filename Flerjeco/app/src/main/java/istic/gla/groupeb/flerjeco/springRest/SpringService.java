package istic.gla.groupeb.flerjeco.springRest;

import android.util.Log;

import org.springframework.http.HttpMethod;
import org.springframework.http.ResponseEntity;
import org.springframework.http.client.HttpComponentsClientHttpRequestFactory;
import org.springframework.http.converter.json.MappingJackson2HttpMessageConverter;
import org.springframework.web.client.HttpServerErrorException;
import org.springframework.web.client.HttpStatusCodeException;
import org.springframework.web.client.ResourceAccessException;
import org.springframework.web.client.RestTemplate;

import java.sql.Timestamp;

import entity.Drone;
import entity.IncidentCode;
import entity.Intervention;
import entity.Resource;
import entity.ResourceType;
import entity.StaticData;

/**
 * Created by amhachi on 08/04/15.
 */
public class SpringService {

    private static final String TAG = SpringService.class.getSimpleName();
    private static final String URL = "http://ns3002211.ip-37-59-58.eu:8080/nivimoju/rest/";
    private static RestTemplate restTemplate = new RestTemplate();

    /**
     * @see istic.gla.groupeb.flerjeco.springRest.SpringService
     * default constructor {@link SpringService}
     */
    public SpringService(){
        restTemplate.setRequestFactory(new HttpComponentsClientHttpRequestFactory());
        restTemplate.getMessageConverters().add(new MappingJackson2HttpMessageConverter());
    }

    /**
     * get resource by id
     * @param idRes id of the resource to get
     * @return the resource type retrieved
     */
    public ResourceType getResourceTypeById(Long idRes){

        final String url = URL + "resource/"+idRes;

        ResponseEntity<ResourceType> resourcetype = restTemplate.getForEntity(url, ResourceType.class);

        ResourceType rt = resourcetype.getBody();
        return rt;
    }

    /**
     * get intervention by id
     *
     * @param idIntervention id of the resource to get
     * @return the intervention retrieved
     */
    public Intervention getInterventionById(Long idIntervention) {

        final String url = URL + "intervention/" + idIntervention;

        ResponseEntity<Intervention> interventionResult = restTemplate.getForEntity(url, Intervention.class);

        Intervention intervention = interventionResult.getBody();
        return intervention;
    }

    /**
     * Get incident codes
     * @return array of {@link entity.IncidentCode}
     * @throws HttpStatusCodeException throw exception if status code is bad
     */
    public IncidentCode[] codeSinistreClient() throws HttpStatusCodeException {

        final String url = URL + "incidentcode";

        ResponseEntity<IncidentCode[]> incidentCode = restTemplate.getForEntity(url, IncidentCode[].class);

        IncidentCode[] codes = incidentCode.getBody();
        return codes;


    }

    /**
     * Creates intervention
     * @param intervention intervention to be created
     * @return The intervention created
     */
    public Intervention postIntervention(Intervention intervention) {
        try {

            final String url = URL + "intervention/create";
            ResponseEntity<Intervention> interventionResult = restTemplate.postForEntity(url, intervention, Intervention.class);

            if (interventionResult == null) {
                Log.i(TAG, "interventionResult = null");
            } else {
                return interventionResult.getBody();
            }
        } catch (HttpStatusCodeException e) {
            Log.i(TAG, "Problème de la création de l'intervention : " + e.getMessage());
        }
        return null;
    }

    /**
     * Assign a drone on an intervention
     * @param id
     * @return
     */
    public Drone assignDrone(Long id){

        // assignement of the drone for the intervention
        final String urlDrone = URL + "drone/assign/" + id;
        Log.i(TAG, urlDrone);
        ResponseEntity<Drone> drone = restTemplate.getForEntity(urlDrone, Drone.class);
        return drone.getBody();
    }

    /**
     * Unassign a drone on an intervention
     * @param id
     * @return
     */
    public Drone unAssignDrone(Long id){

        // unassignement of the drone for the intervention
        final String urlDrone = URL + "drone/unassign/" + id;
        Log.i(TAG, urlDrone);
        ResponseEntity<Drone> drone = restTemplate.getForEntity(urlDrone, Drone.class);
        return drone.getBody();
    }

    /**
     * Updates intervention with the one in parameters
     * @param intervention The new intervention
     * @return The new intervention backing from the server
     */
    public Intervention updateIntervention(Intervention intervention) {
        try {
            final String url = URL + "intervention/update";

            ResponseEntity<Intervention> interventionResult = restTemplate.postForEntity(url, intervention, Intervention.class);

            if (interventionResult == null) {
                Log.i(TAG, "interventionResult = null");
            } else
                Log.i(TAG, interventionResult.toString());
                return interventionResult.getBody();
        } catch (HttpStatusCodeException e) {
            Log.i(TAG, "Fail updating intervention : " + e.getMessage());
        }
        return null;
    }

    public Intervention updateResourceIntervention(Object[] params) {
        long interventionId = (long)params[0];
        Resource resource = (Resource)params[1];
        try {
            final String url = URL + "intervention/"+interventionId+"/resources/update";

            ResponseEntity<Intervention> interventionResult = restTemplate.postForEntity(url, resource, Intervention.class);

            if (interventionResult == null) {
                Log.i(TAG, "updateResourceIntervention interventionResult = null");
            } else {
                Log.i(TAG, "Intervention up to date : "+interventionResult.toString());
            }
            return interventionResult.getBody();
        } catch (HttpStatusCodeException e) {
            Log.i(TAG, "resource can not be update : " + e.getMessage());
        }
        return null;
    }

    /**
     * Tries to log in the user with his id and password
     * @param id The user identifier
     * @param password The user password
     * @return 200 if connected
     */
    public String login(String id, String password) {
        Log.i(TAG, "login start");
        final String url = URL + "authentication/connected/" + id + "/" + password;
        String httpResult = "";
        try {
            ResponseEntity<String> entity = restTemplate.getForEntity(url, String.class);
            httpResult = entity.getStatusCode().toString();
        } catch (HttpStatusCodeException e) {
            httpResult = e.getStatusCode().toString();
        } catch (ResourceAccessException e) {
            httpResult = "500";
        }
        Log.i(TAG, "httpResult : " + httpResult);
        Log.i(TAG, "login end");
        return httpResult;
    }

    /**
     * Gets a notification from server
     * @param url url already prepared to call the server
     * @param timestamp lastUpdate timestamp
     * @return the lastUpdate timestamp
     */
    public Timestamp getNotify(String url, Timestamp timestamp) {
        String httpCode = "";
        Timestamp restTimestamp = timestamp;
        url = URL+url;
        try {
            ResponseEntity<Timestamp> entity = restTemplate.postForEntity(url, timestamp, Timestamp.class);
            httpCode = entity.getStatusCode().toString();
            restTimestamp = entity.getBody();
            Log.i(TAG, "HttpCode  :  " + httpCode);
            if ("201".equals(httpCode)) {
                return restTimestamp;
            }else {
                return timestamp;
            }

        } catch (HttpStatusCodeException e) {
            httpCode = e.getStatusCode().toString();
            return timestamp;
        }

    }


    /**
     * Gets all the resource types
     * @return A list of resource types
     */
    public ResourceType[] resourceTypes() {
        final String url = URL + "resource";
        ResourceType[] resourceTypes = null;

        try {
            ResponseEntity<ResourceType[]> resourceTypesEntity = restTemplate.getForEntity(url, ResourceType[].class);
            resourceTypes = resourceTypesEntity.getBody();
            Log.i(TAG, "resourceTypes StatusCode : " + resourceTypesEntity.getStatusCode().toString());
        } catch (HttpServerErrorException e) {
            Log.e(TAG, e.getMessage());
        }
        return resourceTypes;
    }

    /**
     * Requests a new vehicle for an intervention of a type in resource types
     * @param params The intervention id and the vehicle type
     * @return The updated intervention
     */
    public Intervention requestVehicle(Object[] params) {
        final String url = URL + "intervention/" + params[0] + "/resources/" + params[1];
        Log.i(TAG, "requestVehicle : "+url);
        Intervention intervention = null;
        try {
            ResponseEntity<Intervention> interventionEntity = restTemplate.exchange(url, HttpMethod.PUT, null, Intervention.class);
            intervention = interventionEntity.getBody();
            Log.i(TAG, "requestVehicle StatusCode "+interventionEntity.getStatusCode());
        } catch (HttpServerErrorException e) {
            Log.e(TAG, e.getMessage());
        }

        return intervention;
    }

    /**
     * Gets all the interventions
     * @return An array of interventions
     */
    public Intervention[] getAllInterventions() {
        Log.i(TAG, "getAllInterventions start");
        final String url = URL + "intervention";
        Intervention[] interventions = null;
        try {
            ResponseEntity<Intervention[]> entity = restTemplate.getForEntity(url, Intervention[].class);
            interventions = entity.getBody();
        } catch (ResourceAccessException e) {
            Log.i(TAG, "getAllInterventions : " + e.getLocalizedMessage());
        }
        Log.i(TAG, "getAllInterventions end");
        return interventions;
    }

    /**
     * Gets all the drone for the intervention
     * @return An array of drones
     */
    public Drone[] getAllDroneByIntervention(Object[] params) {
        Log.i(TAG, "getAllDroneByIntervention start");
        final String url = URL + "drone/byIntervention/" + params[0];
        Drone[] drones = null;
        Log.i(TAG, url);
        try {
            ResponseEntity<Drone[]> entity = restTemplate.getForEntity(url, Drone[].class);
            drones = entity.getBody();
        } catch (ResourceAccessException e) {
            Log.i(TAG, "getAllDroneByIntervention : " + e.getLocalizedMessage());
        }
        Log.i(TAG, "getAllDroneByIntervention end");
        return drones;
    }


    public Long moveDrone(Object[] params) {
        Log.i(TAG, "move drone to : " + params[0] + ", " + params[1]);
        final String url = URL + "drone/move/" + params[0] + "/" + params[1];

        ResponseEntity<Long> id = restTemplate.exchange(url, HttpMethod.GET, null, Long.class);
        return id.getBody();
    }

    /**
     * Change the state of a resource in parameters (waiting, planned, validated...)
     * @param params The id of the intervention, the resource label and the new state
     * @return The updated intervention
     */
    public Intervention changeResourceState(Object[] params) {
        final String url = URL + "intervention/" + params[0] + "/resources/" + params[1] + "/" + params[2];
        ResponseEntity<Intervention> intervention = null;
        try {
            intervention = restTemplate.exchange(url, HttpMethod.PUT, null, Intervention.class);
        } catch (HttpServerErrorException e) {
            Log.e(TAG, e.getMessage());
            return null;
        }
        Log.i("SpringService", intervention.getBody().getName());
        return intervention.getBody();
    }

    /**
     * Gets all static data
     * @return A list of static data
     */
    public StaticData[] getAllStaticDatas() {
        Log.i(TAG, "getAllStaticDatas start");
        final String url = URL + "staticdata";
        StaticData[] datas = null;
        try {
            ResponseEntity<StaticData[]> entity = restTemplate.getForEntity(url, StaticData[].class);
            datas = entity.getBody();
        } catch (HttpServerErrorException e) {
            Log.i(TAG, e.getMessage());
            return datas;
        } catch (ResourceAccessException e) {
            Log.i(TAG, "getAllStaticDatas : " + e.getLocalizedMessage());
            return datas;
        }
        Log.i(TAG, "getAllStaticData success");
        return datas;
    }
}
