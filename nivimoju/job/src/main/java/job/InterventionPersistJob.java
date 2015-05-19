package job;

import container.InterventionContainer;
import dao.InterventionDAO;
import entity.Intervention;
import org.apache.log4j.Logger;
import org.quartz.DisallowConcurrentExecution;
import org.quartz.Job;
import org.quartz.JobExecutionContext;
import org.quartz.JobExecutionException;

/**
 * Job refreshing the database with positions from simulation
 */
@DisallowConcurrentExecution
public class InterventionPersistJob implements Job{
    Logger logger = Logger.getLogger(InterventionPersistJob.class);

    @Override
    public void execute(JobExecutionContext jobExecutionContext) throws JobExecutionException {
        logger.trace("starting job to persist intervention");
        InterventionContainer container = InterventionContainer.getInstance();
        InterventionDAO interventionDAO = new InterventionDAO();
        interventionDAO.connect();
        for(Intervention intervention : container.getInterventions()) {
            intervention.updateDate();
            interventionDAO.update(intervention);
        }
        interventionDAO.disconnect();
        logger.trace("ending job to persist intervention");
    }
}
