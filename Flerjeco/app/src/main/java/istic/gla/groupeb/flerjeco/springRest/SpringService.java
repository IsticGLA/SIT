package istic.gla.groupeb.flerjeco.springRest;

import android.util.Log;

import org.springframework.http.HttpEntity;
import org.springframework.http.HttpHeaders;
import com.google.android.gms.maps.model.LatLng;

import org.springframework.http.HttpMethod;
import org.springframework.http.HttpStatus;
import org.springframework.http.MediaType;
import org.springframework.http.ResponseEntity;
import org.springframework.http.client.HttpComponentsClientHttpRequestFactory;
import org.springframework.http.converter.json.MappingJackson2HttpMessageConverter;
import org.springframework.web.client.HttpServerErrorException;
import org.springframework.web.client.HttpStatusCodeException;
import org.springframework.web.client.ResourceAccessException;
import org.springframework.web.client.RestTemplate;

import java.sql.Timestamp;
import java.util.Locale;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;

import istic.gla.groupb.nivimoju.customObjects.TimestampedPosition;
import istic.gla.groupb.nivimoju.entity.Drone;
import istic.gla.groupb.nivimoju.entity.Image;
import istic.gla.groupb.nivimoju.entity.IncidentCode;
import istic.gla.groupb.nivimoju.entity.Intervention;
import istic.gla.groupb.nivimoju.entity.Path;
import istic.gla.groupb.nivimoju.entity.Resource;
import istic.gla.groupb.nivimoju.entity.ResourceType;
import istic.gla.groupb.nivimoju.entity.StaticData;
import istic.gla.groupeb.flerjeco.agent.planZone.EPathOperation;

/**
 * Created by amhachi on 08/04/15.
 */
public class SpringService {

    private static final String TAG = SpringService.class.getSimpleName();
    private static final String URL = "http://ns3002211.ip-37-59-58.eu:8080/nivimoju/rest/";
    private RestTemplate restTemplate;

    /**
     * @see istic.gla.groupeb.flerjeco.springRest.SpringService
     * default constructor {@link SpringService}
     */
    public SpringService(){
        restTemplate = new RestTemplate();
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
        ResourceType rt = null;

        try {
            ResponseEntity<ResourceType> resourcetype = restTemplate.getForEntity(url, ResourceType.class);
            rt = resourcetype.getBody();
            Log.v(TAG, "getResourceTypeById : " + resourcetype.getStatusCode().toString());
        } catch (Throwable e) {
            Log.e(TAG, e.getMessage());
        }

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
        Intervention intervention = null;

        try {
            ResponseEntity<Intervention> interventionResult = restTemplate.getForEntity(url, Intervention.class);
            intervention = interventionResult.getBody();
            Log.v(TAG, "getInterventionById : " + interventionResult.getStatusCode().toString());
        } catch (Throwable e) {
            Log.e(TAG, e.getMessage());
        }

        return intervention;
    }

    /**
     * Get incident codes
     * @return array of {@link IncidentCode}
     * @throws HttpStatusCodeException throw exception if status code is bad
     */
    public IncidentCode[] codeSinistreClient() throws HttpStatusCodeException {
        final String url = URL + "incidentcode";
        IncidentCode[] codes = null;

        try {
            ResponseEntity<IncidentCode[]> incidentCode = restTemplate.getForEntity(url, IncidentCode[].class);
            codes = incidentCode.getBody();
            Log.v(TAG, "codeSinistreClient : " + incidentCode.getStatusCode().toString());
        } catch (Throwable e) {
            Log.e(TAG, e.getMessage());
        }

        return codes;
    }

    /**
     * Creates intervention
     * @param intervention intervention to be created
     * @return The intervention created
     */
    public Intervention postIntervention(Intervention intervention) {
        final String url = URL + "intervention/create";
        Intervention inter = null;

        try {
            ResponseEntity<Intervention> interventionResult = restTemplate.postForEntity(url, intervention, Intervention.class);
            inter = interventionResult.getBody();
            Log.v(TAG, "postIntervention : " + interventionResult.getStatusCode().toString());
        } catch (Throwable e) {
            Log.e(TAG, e.getMessage());
        }

        return inter;
    }

    /**
     * Updates intervention with the one in parameters
     * @param intervention The new intervention
     * @return The new intervention backing from the server
     */
    public Intervention updateIntervention(Intervention intervention) {
        final String url = URL + "intervention/update";
        Intervention inter = null;

        try {
            ResponseEntity<Intervention> interventionResult = restTemplate.postForEntity(url, intervention, Intervention.class);
            inter = interventionResult.getBody();
            Log.v(TAG, interventionResult.getStatusCode().toString());
            Log.v(TAG, "interventionResult : "+interventionResult.toString());
            return inter;
        } catch (Throwable e) {
            Log.e(TAG, e.getMessage());
        }
        return inter;
    }

    public Intervention updateResourceIntervention(long interventionId, Resource resource) {
        final String url = URL + "intervention/"+interventionId+"/resources/update";
        Intervention intervention = null;

        try {
            ResponseEntity<Intervention> interventionResult = restTemplate.postForEntity(url, resource, Intervention.class);
            intervention = interventionResult.getBody();
            Log.v(TAG, "updateResourceIntervention : "+interventionResult.getStatusCode().toString());
            if (interventionResult == null) {
                Log.v(TAG, "updateResourceIntervention interventionResult = null");
            } else {
                Log.v(TAG, "Intervention up to date : "+interventionResult.toString());
            }
        } catch (Throwable e) {
            Log.e(TAG, e.getMessage());
        }

        return intervention;
    }

    /**
     * Tries to log in the user with his id and password
     * @param id The user identifier
     * @param password The user password
     * @return 200 if connected
     */
    public String login(String id, String password) {
        final String url = URL + "authentication/connected/" + id + "/" + password;
        Log.v(TAG, "login start");
        String httpResult = "";

        try {
            ResponseEntity<String> entity = restTemplate.getForEntity(url, String.class);
            httpResult = entity.getStatusCode().toString();
        } catch (HttpStatusCodeException e) {
            httpResult = "login : "+e.getStatusCode().toString();
        } catch (ResourceAccessException e) {
            httpResult = "500";
        } catch (Throwable e) {
            Log.e(TAG, e.getMessage());
        }

        Log.v(TAG, "httpResult : " + httpResult);
        Log.v(TAG, "login end");
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
        Log.v(TAG, url);

        try {
            ResponseEntity<Timestamp> entity = restTemplate.postForEntity(url, timestamp, Timestamp.class);
            httpCode = entity.getStatusCode().toString();
            restTimestamp = entity.getBody();
            Log.v(TAG, "HttpCode  :  " + httpCode);
            if ("201".equals(httpCode)) {
                return restTimestamp;
            }
        } catch (HttpStatusCodeException e) {
            httpCode = e.getStatusCode().toString();
            return timestamp;
        } catch (Throwable e) {
            Log.e(TAG, e.getMessage());
        }

        return timestamp;
    }


    /**
     * Gets all the resource types
     * @return A list of resource types
     */
    public ResourceType[] getResourceTypes() {
        final String url = URL + "resource";
        ResourceType[] resourceTypes = null;

        try {
            ResponseEntity<ResourceType[]> resourceTypesEntity = restTemplate.getForEntity(url, ResourceType[].class);
            resourceTypes = resourceTypesEntity.getBody();
            Log.v(TAG, "getResourceTypes StatusCode : " + resourceTypesEntity.getStatusCode().toString());
        } catch (HttpServerErrorException e) {
            Log.e(TAG, e.getMessage());
        } catch (Throwable e) {
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
        Log.v(TAG, "requestVehicle : " + url);
        Intervention intervention = null;

        try {
            ResponseEntity<Intervention> interventionEntity = restTemplate.exchange(url, HttpMethod.PUT, null, Intervention.class);
            intervention = interventionEntity.getBody();
            Log.v(TAG, "requestVehicle StatusCode "+interventionEntity.getStatusCode());
        } catch (HttpServerErrorException e) {
            Log.e(TAG, e.getMessage());
        } catch (Throwable e) {
            Log.e(TAG, e.getMessage());
        }

        return intervention;
    }

    /**
     * Gets all the interventions
     * @return a responseEntity containing an array of interventions
     */
    public ResponseEntity<Intervention[]> getAllInterventions() {
        Log.v(TAG, "Getting all interventions");
        final String url = URL + "intervention";

        try {
            ResponseEntity<Intervention[]> entity = restTemplate.getForEntity(url, Intervention[].class);
            return entity;
        } catch (ResourceAccessException|HttpServerErrorException e) {
            Log.e(TAG, "getAllInterventions : " + e.getLocalizedMessage());
        } catch (Throwable e) {
            Log.e(TAG, "failed to get interventions", e);
        }
        return new ResponseEntity<Intervention[]>(HttpStatus.INTERNAL_SERVER_ERROR);
    }

    /**
     * Gets all the drone for the intervention
     * @return An array of drones
     */
    public ResponseEntity<Drone[]> getAllDroneByIntervention(Long interventionId) {
        Log.v(TAG, "getAllDroneByIntervention start");
        final String url = URL + "drone/byIntervention/" + interventionId;
        return restTemplate.getForEntity(url, Drone[].class);
    }

    /**
     * upodate the paths of an intervention
     * @param obj
     * @return
     */
    public ResponseEntity<Intervention> updateInterventionPaths(Object[] obj, EPathOperation operation) {
        String url = null;
        if (operation == EPathOperation.CREATE){
            url = URL + "intervention/" + obj[0] + "/watchpath/create";
        } else if (operation == EPathOperation.UPDATE) {
            url = URL + "intervention/" + obj[0] + "/watchpath/update";
        } else {
            url = URL + "intervention/" + obj[0] + "/watchpath/delete";
        }
        try {
            return restTemplate.postForEntity(url, obj[1], Intervention.class);
        } catch (HttpServerErrorException e){
            Log.e(TAG, "erreur à l'update d'un path", e);
            throw e;
        }
    }

    /**
     * Gets all the image for the intervention and position
     * @return An array of drones
     */
    public ResponseEntity<Image[]> getAllImageForInterventionAndPosition(Long interventionId, LatLng position, long timestamp) {
        final String url = URL + "image/all/" + interventionId + "/" + position.latitude + "/" + position.longitude + "/" + timestamp;
        Log.v(TAG, "getAllImageForInterventionAndPosition on " + url);
        return restTemplate.getForEntity(url, Image[].class);
    }

    /**
     * Gets the last image taken by a drone
     * @return an image
     */
    public ResponseEntity<Image> getLastImageForDrone(String droneLabel) {
        final String url = URL + "image/video/" + droneLabel;
        Log.v(TAG, "getLastImageForDrone on " + url);
        return restTemplate.getForEntity(url, Image.class);
    }

    /**
     * Change the state of a resource in parameters (waiting, planned, validated...)
     * @param params The id of the intervention, the resource label and the new state
     * @return The updated intervention
     */
    public Intervention changeResourceState(Object[] params) {
        final String url = URL + "intervention/" + params[0] + "/resources/" + params[1] + "/" + params[2];
        Log.i(TAG, "changeResourceState URL : " + url);
        Intervention intervention = null;

        try {
            ResponseEntity<Intervention> interventionEntity = restTemplate.exchange(url, HttpMethod.PUT, null, Intervention.class);
            intervention = interventionEntity.getBody();
            Log.d(TAG, "changeResourceState : " + interventionEntity.getStatusCode().toString());
            Log.d("SpringService", interventionEntity.getBody().getName());
        } catch (HttpServerErrorException e) {
            Log.e(TAG, e.getMessage());
            return null;
        } catch (Throwable e) {
            Log.e(TAG, e.getMessage());
        }

        return intervention;
    }

    /**
     * Gets all static data
     * @return A list of static data
     */
    public StaticData[] getAllStaticDatas() {
        Log.v(TAG, "getAllStaticDatas start");
        final String url = URL + "staticdata";
        StaticData[] datas = null;

        try {
            ResponseEntity<StaticData[]> entity = restTemplate.getForEntity(url, StaticData[].class);
            datas = entity.getBody();
            Log.v(TAG, "getAllStaticDatas : " + entity.getStatusCode());
        } catch (HttpServerErrorException e) {
            Log.v(TAG, "getAllStaticDatas : " + e.getMessage());
        } catch (ResourceAccessException e) {
            Log.v(TAG, "getAllStaticDatas : " + e.getLocalizedMessage());
        } catch (Throwable e) {
            Log.e(TAG, e.getMessage());
        }

        return datas;
    }

    public Image[] getLastImages(Long interventionId, List<TimestampedPosition> timestampedPositions) {
        Log.v(TAG, "getLastImages start");
        final String url = URL + "image/last/"+ interventionId;
        Image[] images = null;

        try {
            HttpHeaders headers = new HttpHeaders();
            headers.setContentType(MediaType.APPLICATION_JSON);
            HttpEntity<List<TimestampedPosition>> timestamp = new HttpEntity<>(timestampedPositions, headers);
            ResponseEntity<Image[]> entity = restTemplate.postForEntity(url, timestamp, Image[].class);
            images = entity.getBody();
            Log.v(TAG, "getLastImages: " + entity.getStatusCode());
        } catch (HttpServerErrorException e) {
            Log.v(TAG, "getLastImages: " + e.getMessage());
        } catch (ResourceAccessException e) {
            Log.v(TAG, "getLastImages: " + e.getLocalizedMessage());
        } catch (Throwable e) {
            Log.e(TAG, e.getMessage());
        }

        return images;
    }
}
