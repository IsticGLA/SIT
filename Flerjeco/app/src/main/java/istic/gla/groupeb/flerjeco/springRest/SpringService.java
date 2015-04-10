package istic.gla.groupeb.flerjeco.springRest;

import android.util.Log;

import org.springframework.http.HttpMethod;
import org.springframework.http.ResponseEntity;
import org.springframework.http.converter.json.MappingJackson2HttpMessageConverter;
import org.springframework.web.client.HttpStatusCodeException;
import org.springframework.web.client.RestTemplate;

import java.util.List;

import entity.IncidentCode;
import entity.Intervention;
import entity.ResourceType;

/**
 * Created by amhachi on 08/04/15.
 */
public class SpringService {
    private static final String TAG = SpringService.class.getSimpleName();
    private static final String URL = "http://ns3002211.ip-37-59-58.eu:8080/nivimoju/rest/";

    boolean test = true;
    public IncidentCode[]  codeSinistreClient() throws HttpStatusCodeException{


        final String url = URL + "incidentcode";


            RestTemplate restTemplate = new RestTemplate();
            restTemplate.getMessageConverters().add(new MappingJackson2HttpMessageConverter());

            ResponseEntity<IncidentCode[]> incidentCode = restTemplate.getForEntity(url, IncidentCode[].class);
            IncidentCode[] codes = incidentCode.getBody();
            return codes;


    }


    public long  postIntervention(Intervention intervention){
       try {

           final String url = URL + "intervention/create";

           RestTemplate restTemplate = new RestTemplate();
           restTemplate.getMessageConverters().add(new MappingJackson2HttpMessageConverter());
           Intervention intervetionResult = restTemplate.postForObject(url, intervention, Intervention.class);

           return intervention.getId();
       }catch (HttpStatusCodeException e){
           Log.i("MAMH","Problème de la création de l'intervention : "+e.getMessage());
       }
        return  0;
    }

    public String login(String id, String password) {
        Log.i(TAG, "login start");
        final String url = URL + "authentication/connected/"+id+"/"+password;
        String httpResult = "";

        RestTemplate restTemplate = new RestTemplate();
        restTemplate.getMessageConverters().add(new MappingJackson2HttpMessageConverter());

        try {
            ResponseEntity<String> entity = restTemplate.getForEntity(url, String.class);
            httpResult = entity.getStatusCode().toString();
        } catch (HttpStatusCodeException e) {
            httpResult = "400";
        }
        Log.i(TAG, "httpResult : "+httpResult);
        Log.i(TAG, "login end");
        return httpResult;
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
        restTemplate.getMessageConverters().add(new MappingJackson2HttpMessageConverter());

        ResponseEntity<Intervention[]> entity = restTemplate.getForEntity(url, Intervention[].class);
        Log.i(TAG, "getAllInterventions : "+ entity.getBody().toString());
        return entity.getBody();
    }
}
