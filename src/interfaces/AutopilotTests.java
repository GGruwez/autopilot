package interfaces;

import static org.junit.Assert.*;

import org.junit.Before;
import org.junit.Test;

public class AutopilotTests {
	
	AutopilotModuleImplementation module;
	Airport airport0;
	Airport airport1;
	Airport airport2;
	Airport airport3;
	AutopilotImplementation drone0;
	AutopilotImplementation drone1;
	
	@Before
	public void before() {
		module = new AutopilotModuleImplementation();
		module.defineAirport(0,0,0,-1);
		module.defineAirport(2000,0,0,-1);
		module.defineAirport(0,1500,0,-1);
		module.defineAirport(2000,1500,0,-1);
		
		airport0 = module.getAirport(0);
		airport1 = module.getAirport(1);
		airport2 = module.getAirport(2);
		airport3 = module.getAirport(3);
		
		AutopilotConfig config = null;
		module.defineDrone(0, 0, 0, config);
		module.defineDrone(2, 0, 0, config);
		
		drone0 = module.getDrone(0);
		drone1 = module.getDrone(1);
	}

	@Test
	public void assignJobs() {
		assertFalse(drone0.hasJob());
		assertFalse(drone1.hasJob());
		assertTrue(drone1.getDrone().getAirport() == airport2);
		
		module.deliverPackage(0, 0, 1, 0);
		assertTrue(drone0.hasJob());
		assertTrue(module.getJobs().size() == 1);
		assertTrue(drone0.getCurrentJob().getAirportTo() == airport1);
		assertTrue(drone0.getCurrentJob().getAirportFrom() == airport0);
		
		module.deliverPackage(1, 0, 2, 0);
		assertTrue(drone1.hasJob());
		assertTrue(drone1.getCurrentPath()[0] == airport2);
		assertTrue(module.getJobs().size() == 2);
		assertTrue(drone1.getDrone().getAirport() == airport2);
		assertTrue(drone1.getCurrentJob().getAirportTo() == airport1);
	}
	
	@Test 
	public void TestPathCreation() {
		module.deliverPackage(0, 0, 1, 0);
		assertTrue(drone0.hasJob());
		assertTrue(drone0.getCurrentJob().calculatePath() != null);
	}
}
