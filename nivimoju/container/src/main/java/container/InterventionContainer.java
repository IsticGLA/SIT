package container;

import dao.InterventionDAO;
import entity.Intervention;
import entity.Resource;
import org.apache.log4j.Logger;
import util.State;

import java.util.*;

/**
 * Aggregation class that store the interventions for quick access
 *
 * This class handles all modifications of interventions data and is the reference for every read operation
 */
public class InterventionContainer {
    Logger logger = Logger.getLogger(InterventionContainer.class);
    private static InterventionContainer instance;

    //the main container, used for read and write operations on drones
    private Map<Long, Intervention> mapInterventionById;

    /**
     * Initialise the InterventionContainer with data from DB
     */
    private InterventionContainer(){
        mapInterventionById = new HashMap<>();
        logger.info("updating interventions from database");
        InterventionDAO interventionDAO = new InterventionDAO();
        interventionDAO.connect();
        List<Intervention> interventions = interventionDAO.getAll();
        interventionDAO.disconnect();
        if(interventions != null && interventions.size() > 0) {
            logger.info("got " + interventions.size() + " interventions from database");
            loadDrones(interventions);
        } else{
            logger.error("failed to load drones");
        }
    }

    public static synchronized InterventionContainer getInstance(){
        if(instance == null){
            instance = new InterventionContainer();
        }
        return instance;
    }

    /**
     * charge une liste d'intervention et prepare les maps interne
     * @param interventions la liste d'interventions a charger
     **/
    protected void loadDrones(List<Intervention> interventions){
        logger.info("loading interventions internally");
        mapInterventionById.clear();
        for(Intervention intervention : interventions) {
            logger.info("loading intervention[" + intervention.getName() + "]");
            //update des listes interne
            mapInterventionById.put(intervention.getId(), intervention);
        }
        logger.info("loaded " + mapInterventionById.size() + " drones : " + mapInterventionById.keySet());
    }

    protected Map<Long, Intervention> getMapInterventionById(){
        return mapInterventionById;
    }

    protected static void destroy(){
        instance = null;
    }

    /**
     * Met un jour une intervention
     * @param intervention update l'intervention
     */
    public Intervention updateIntervention(Intervention intervention){
        Intervention interventionMap = mapInterventionById.get(intervention.getId());
        if(interventionMap != null) {
            mapInterventionById.put(intervention.getId(), intervention);
        } else{
            logger.error("We got info for intervention [" + intervention.getId() +
                    "] but it does not seem to exist (existing ids : " + mapInterventionById.keySet() + ")");
        }
        return interventionMap;
    }

    /**
     * Accede a l'ensemble des interventions
     * @return toutes les interventions
     */
    public Collection<Intervention> getInterventions(){
        return mapInterventionById.values();
    }

    /**
     * Accede a l'ensemble des interventions
     * @param id
     * @return l'intervention d'id id
     */
    public Intervention getInterventionById(Long id){
        return mapInterventionById.get(id);
    }

    /**
     * Accede a l'ensemble des interventions
     * @param intervention
     * @return Cree l'intervention intervention
     */
    public Intervention createIntervention(Intervention intervention){
        InterventionDAO interventionDAO= new InterventionDAO();
        interventionDAO.connect();
        Intervention resultat = interventionDAO.create(intervention);
        interventionDAO.disconnect();
        mapInterventionById.put(resultat.getId(), resultat);

        return intervention;
    }

    /**
     * Change state of a ressource
     * @param id
     * @param res
     * @param state
     * @return
     */
    public Intervention changeResourceState(Long id, Long res, String state) {
        Intervention intervention = getInterventionById(id);
        for (Resource resource : intervention.getResources()) {
            if (resource.getIdRes() == res) {
                resource.setState(State.valueOf(state));
                break;
            }
        }
        return intervention;
    }

    /**
     * Add Ressource to an intervention
     * @param idintervention
     * @param vehicleName
     * @return
     */
    public Intervention addResource(Long idintervention, String vehicleName) {
        Intervention intervention = getInterventionById(idintervention);
        Long id = Long.valueOf(0);
        for(Resource resource : intervention.getResources()) {
            if(resource.getIdRes() >= id) {
                id = resource.getIdRes();
            }
        }
        id++;
        intervention.getResources().add(new Resource(id, vehicleName + id, State.waiting));

        return intervention;
    }

    /**
     * Add Ressource to an intervention
     * @param idintervention
     * @param vehicle
     * @return
     */
    public Intervention placeVehicle(Long idintervention, Resource vehicle) {
        boolean found = false;
        Intervention intervention = getInterventionById(idintervention);
        for (Resource resource : intervention.getResources()) {
            if (resource.getIdRes() == vehicle.getIdRes()) {
                resource = vehicle;
                found = true;
            }
        }
        if (!found){
            intervention.getResources().add(vehicle);
        }

        return intervention;
    }
}
