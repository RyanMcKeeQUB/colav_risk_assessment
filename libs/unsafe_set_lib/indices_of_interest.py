from collections import defaultdict
import numpy as np

class IndicesOfInterest:
    def __init__(self, planning_problem, trajectory, obstacles_map):
        """
        Constructor for the IndicesOfInterest class.
        :param planning_problem: Dict containing simulation parameters (timesteps, dt)
        :param trajectory: List of dicts representing the LOS trajectory
        :param obstacles_map: Dict containing obstacle details
        """
        self.I1 = None
        self.I2 = None
        self.I3 = None
        self.uIoI = None
        
        timesteps = len(trajectory)
        self.I1, self.I2, self.I3 = self.calculate_indices_of_interest(trajectory, obstacles_map, planning_problem)
        self.uIoI = self.unionise_indices_of_interest(self.I1, self.I2, self.I3, timesteps)
    
    def calculate_indices_of_interest(self, trajectory, obstacles_map, planning_problem):
        timesteps = len(trajectory)
        if obstacles_map:
            I1 = self.calculate_I1(trajectory, obstacles_map, timesteps)
            I2 = self.calculate_I2(obstacles_map, timesteps)
            I3 = self.calculate_I3(trajectory, obstacles_map, timesteps, planning_problem['dt'])
            return I1, I2, I3
        return {}, {}, {}

    def unionise_indices_of_interest(self, I1, I2, I3, timesteps):
        I_map = defaultdict(list)
        for timestep in range(timesteps):
            all_obstacles = set(I1.get(timestep, []) + I2.get(timestep, []) + I3.get(timestep, []))
            I_map[timestep] = list(all_obstacles)
        return I_map
    
    def calculate_I1(self, trajectory, obstacles_map, timesteps):
        from motion_planner.common.utils import Utils
        from motion_planner.path_planner import ObservationParams
        
        I1 = defaultdict(list)
        
        for timestep in range(timesteps):
            risk_obstacles = []
            los_pose = np.array([trajectory[timestep]['x'], trajectory[timestep]['y']])
            
            for obstacle_key, obstacle_data in obstacles_map.items():
                obstacle = obstacle_data['obstacle']
                if timestep in obstacle.trajectory.trajectory_map:
                    obs_pose = np.array([obstacle.trajectory.trajectory_map[timestep]['x'],
                                         obstacle.trajectory.trajectory_map[timestep]['y']])
                    dist = Utils.calculate_euclidean_distance(los_pose, obs_pose)
                    
                    if dist <= ObservationParams.A_DISTANCE:
                        risk_obstacles.append(obstacle)
            
            I1[timestep] = risk_obstacles
        return I1
    
    def calculate_I2(self, obstacles_map, timesteps):
        from motion_planner.common.utils import Utils
        from motion_planner.path_planner import ObservationParams
        
        I2 = defaultdict(list)
        obstacle_keys = list(obstacles_map.keys())
        
        for timestep in range(timesteps):
            unsafe_obstacles = []
            
            for i, key1 in enumerate(obstacle_keys):
                obstacle1 = obstacles_map[key1]['obstacle']
                if timestep not in obstacle1.trajectory.trajectory_map:
                    continue
                pos1 = np.array([obstacle1.trajectory.trajectory_map[timestep]['x'],
                                 obstacle1.trajectory.trajectory_map[timestep]['y']])
                
                for j, key2 in enumerate(obstacle_keys):
                    if i == j:
                        continue
                    obstacle2 = obstacles_map[key2]['obstacle']
                    if timestep not in obstacle2.trajectory.trajectory_map:
                        continue
                    pos2 = np.array([obstacle2.trajectory.trajectory_map[timestep]['x'],
                                     obstacle2.trajectory.trajectory_map[timestep]['y']])
                    
                    dist = Utils.calculate_euclidean_distance(pos1, pos2)
                    if dist <= ObservationParams.SAFE_DISTANCE:
                        unsafe_obstacles.append(obstacle1)
                        break
            
            I2[timestep] = unsafe_obstacles
        return I2

    def calculate_I3(self, trajectory, obstacles_map, timesteps, dt):
        """
        Computes the set of unsafe obstacles for each timestep.
        
        Parameters:
        - trajectory: List of trajectory points containing x and y positions.
        - obstacles_map: Dictionary containing obstacle data, where each obstacle has a trajectory.
        - timesteps: Number of timesteps to evaluate.
        - dt: Time increment for each timestep (used for TCPA calculation).
        
        Returns:
        - I3: Dictionary where each key is a timestep, and the value is a list of 
              unsafe obstacles that are within a specified distance threshold from 
              the LOS trajectory at that timestep.
        """
        from motion_planner.path_planner.observation_params import ObservationParams
        from motion_planner.common.utils import Utils
        
        # Initialize a dictionary to hold unsafe obstacles
        I3_map = defaultdict(list)
        
        # Loop through each obstacle in the obstacles_map
        for obstacle_key, obstacle_map in obstacles_map.items():
            obstacle = obstacle_map['obstacle']
            TCPA = obstacle_map['TCPA']
            
            # Round TCPA to the nearest dt and convert to corresponding timestep index
            TCPA = round(TCPA / dt) * dt
            timesteps_to_TCPA = int(TCPA / dt)
            
            # Ensure the calculated TCPA timestep is within valid range
            if 0 < timesteps_to_TCPA <= timesteps:
                obstacle_coords_at_tcpa = (
                    obstacle.trajectory.trajectory_map[timesteps_to_TCPA].x,
                    obstacle.trajectory.trajectory_map[timesteps_to_TCPA].y
                )
                
                # Evaluate distance from LOS trajectory to obstacle's position at TCPA for each timestep
                for timestep in range(1, timesteps + 1):
                    trajectory_timestep_pose = (trajectory[timestep - 1].x, trajectory[timestep - 1].y)
                    
                    # Compute Euclidean distance
                    distance = Utils.calculate_euclidean_distance(trajectory_timestep_pose, obstacle_coords_at_tcpa)
                    
                    # Check if the distance is within the defined threshold
                    distance_threshold = ObservationParams.A_DISTANCE
                    if distance <= distance_threshold:
                        I3_map[timestep].append(obstacle)
        
        return I3_map