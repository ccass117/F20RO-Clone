from controller import Supervisor
from controller import Keyboard
from controller import Display
import math
import numpy as np
import ga,os,sys,struct

class SupervisorGA:
    def __init__(self):
        # Simulation Parameters
        # Please, do not change these parameters
        self.time_step = 32 # ms
        self.time_experiment = 60 # s
        
        # Initiate Supervisor Module
        self.supervisor = Supervisor()
        # Check if the robot node exists in the current world file
        self.robot_node = self.supervisor.getFromDef("Controller")
        if self.robot_node is None:
            sys.stderr.write("No DEF Controller node found in the current world file\n")
            sys.exit(1)
        # Get the robots translation and rotation current parameters    
        self.trans_field = self.robot_node.getField("translation")  
        self.rot_field = self.robot_node.getField("rotation")
        
        # Check Receiver and Emitter are enabled
        self.emitter = self.supervisor.getDevice("emitter")
        self.receiver = self.supervisor.getDevice("receiver")
        self.receiver.enable(self.time_step)
        
        # Initialize the receiver and emitter data to null
        self.receivedData = "" 
        self.receivedWeights = "" 
        self.receivedFitness = "" 
        self.emitterData = ""
       
       
        
        
        ###########
        ### DEFINE here the 3 GA Parameters:
        self.num_generations = 50
        self.num_population = 10
        self.num_elite = 2
        
        # size of the genotype variable
        self.num_weights = 0
        
        # Creating the initial population
        self.population = []
        
        # All Genotypes
        self.genotypes = []
        
        # Display: screen to plot the fitness values of the best individual and the average of the entire population
        self.display = self.supervisor.getDevice("display")
        self.width = self.display.getWidth()
        self.height = self.display.getHeight()
        self.prev_best_fitness = 0.0;
        self.prev_average_fitness = 0.0;
        self.display.drawText("Fitness (Best - Red)", 0,0)
        self.display.drawText("Fitness (Average - Green)", 0,10)
        
         # Spotlight node
        self.spotlight_node = self.supervisor.getFromDef("Spotlight")
        if self.spotlight_node is None:
            sys.stderr.write("No DEF Spotlight node found in the current world file\n")
            sys.exit(1)
        self.spotlight_position_field = self.spotlight_node.getField("location")
        self.spotlight_color_field = self.spotlight_node.getField("color") 
        
    def createRandomPopulation(self):
        # Wait until the supervisor receives the size of the genotypes (number of weights)
        if(self.num_weights > 0):
            #  Size of the population and genotype
            pop_size = (self.num_population,self.num_weights)
            # Create the initial population with random weights
            self.population = np.random.uniform(low=-1.0, high=1.0, size=pop_size)

    def handle_receiver(self): 
        while self.receiver.getQueueLength() > 0:
            # Webots 2022: 
            # self.receivedData = self.receiver.getData().decode("utf-8")
            # Webots 2023: 
            self.receivedData = self.receiver.getString()
            
            # Extract message type
            typeMessage = self.receivedData[0:7]
            
            # Check message type and process accordingly
            if typeMessage == "weights":
                self.receivedWeights = self.receivedData[9:len(self.receivedData)] 
                self.num_weights = int(self.receivedWeights)
            
            elif typeMessage == "fitness":  
                self.receivedFitness = float(self.receivedData[9:len(self.receivedData)])
            
            elif self.receivedData == "light_on" and self.reward_zone != [0.3, 0.0, -0.15]:
                # Update the reward zone value when light is detected
                self.reward_zone = [0.3, 0.0, -0.15]
                print(f"Reward zone updated to: {self.reward_zone}")
            
            # Move to the next packet
            self.receiver.nextPacket()
            
            
        
    def handle_emitter(self):
        if(self.num_weights > 0):
            # Send genotype of an individual
            string_message = str(self.emitterData)
            string_message = string_message.encode("utf-8")
            #print("Supervisor send:", string_message)
            self.emitter.send(string_message)     
            distance_message = str(self.calculate_distance_to_reward_zone(self.trans_field.getSFVec3f()))
            distance_message = distance_message.encode("utf-8")
            self.emitter.send(distance_message)
        
        
    def run_seconds(self,seconds):
        #print("Run Simulation")
        stop = int((seconds*1000)/self.time_step)
        iterations = 0
        while self.supervisor.step(self.time_step) != -1:
            self.handle_emitter()
            self.handle_receiver()
            #####
            robot_position = self.trans_field.getSFVec3f()
            distance = self.calculate_distance_to_reward_zone(robot_position)
            #print(f"Distance: {distance}")
            #####
            if(stop == iterations):
                break    
            iterations = iterations + 1
            
    def calculate_distance_to_reward_zone(self, robot_position):
        """
        Calculate the Euclidean distance from the robot to the reward zone.
        """
        reward_zone_position = self.reward_zone
        distance = math.sqrt(
            (robot_position[0] - reward_zone_position[0])**2 +
            (robot_position[2] - reward_zone_position[2])**2
        )
        return distance
        
        
                
    def evaluate_genotype(self,genotype,generation):
        # Here you can choose how many times the current individual will interact with both environments
        # At each interaction loop, one trial on each environment will be performed
        numberofInteractionLoops = 3
        currentInteraction = 0
        fitnessPerTrial = []
        while currentInteraction < numberofInteractionLoops:
            #######################################
            # TRIAL: TURN RIGHT
            #######################################
            # Send genotype to robot for evaluation
            self.emitterData = str(genotype)
            
            # Reset robot position and physics
            INITIAL_TRANS = [0.007, 0, 0.35]
            self.trans_field.setSFVec3f(INITIAL_TRANS)
            INITIAL_ROT = [-0.5, 0.5, 0.5, 2.09]
            self.rot_field.setSFRotation(INITIAL_ROT)
            self.robot_node.resetPhysics()
            
            # Reset the spotlight position and physics

            #Webots 2023:
            INITIAL_TRANS = [0.06, 0.02, 0.01]
            self.spotlight_position_field.setSFVec3f(INITIAL_TRANS)
            self.spotlight_node.resetPhysics()
            
            # Set the reward zone initally left
            self.reward_zone = [-0.3, 0.0, -0.15]
        
            # Evaluation genotype 
            self.run_seconds(self.time_experiment)
        
            # Measure fitness
            fitness = self.receivedFitness
            
            # Check for Reward and add it to the fitness value here
            # Add your code here
            
            print("Fitness: {}".format(fitness))     
                        
            # Add fitness value to the vector
            fitnessPerTrial.append(fitness)
            
            #######################################
            # TRIAL: TURN LEFT
            #######################################
            # Send genotype to robot for evaluation
            self.emitterData = str(genotype)
            
            # Reset robot position and physics
            INITIAL_TRANS = [0.007, 0, 0.35]
            self.trans_field.setSFVec3f(INITIAL_TRANS)
            INITIAL_ROT = [-0.5, 0.5, 0.5, 2.09]
            self.rot_field.setSFRotation(INITIAL_ROT)
            self.robot_node.resetPhysics()
            
            # Reset the black spotlight position and physics
            leftPos = [0.06, -10.04, 0.01]
            self.spotlight_position_field.setSFVec3f(leftPos)
            self.spotlight_node.resetPhysics()
        
            # Set the reward zone initally
            self.reward_zone = [-0.3, 0.0, -0.15]
            
            
            # Evaluation genotype 
            self.run_seconds(self.time_experiment)
        
            # Measure fitness
            fitness = self.receivedFitness
            
            # Check for Reward and add it to the fitness value here
            # Add your code here
            
            print("Fitness: {}".format(fitness))
            
            # Add fitness value to the vector
            fitnessPerTrial.append(fitness)
            
            # End 
            currentInteraction += 1
            
        print(fitnessPerTrial)    
        
        fitness = np.mean(fitnessPerTrial)
        current = (generation,genotype,fitness)
        self.genotypes.append(current)  
        
        return fitness

    def run_demo(self):
        # Read File
        genotype = np.load("Best.npy")
        
        # Turn Left
        
        # Send Genotype to controller
        self.emitterData = str(genotype) 
        
        # Reset robot position and physics
        INITIAL_TRANS = [0.007, 0, 0.35]
        self.trans_field.setSFVec3f(INITIAL_TRANS)
        INITIAL_ROT = [-0.5, 0.5, 0.5, 2.09]
        self.rot_field.setSFRotation(INITIAL_ROT)
        self.robot_node.resetPhysics()
        
        # Reset the spotlight position and physics
        INITIAL_TRANS = [0.01, 0.02, 0.193]
        self.spotlight_position_field.setSFVec3f(INITIAL_TRANS)
        self.spotlight_node.resetPhysics()
    
        # Evaluation genotype 
        self.run_seconds(self.time_experiment) 
        
        # Measure fitness
        fitness = self.receivedFitness
        print("Fitness without reward or penalty: {}".format(fitness))
        
        # Turn Right
        
        # Send Genotype to controller
        self.emitterData = str(genotype) 
        
        # Reset robot position and physics
        INITIAL_TRANS = [0.007, 0, 0.35]
        self.trans_field.setSFVec3f(INITIAL_TRANS)
        INITIAL_ROT = [-0.5, 0.5, 0.5, 2.09]
        self.rot_field.setSFRotation(INITIAL_ROT)
        self.robot_node.resetPhysics()
        
        # Reset the spotlight position and physics
        #INITIAL_TRANS = [0.01, -0.03425, 0.193]
        INITIAL_TRANS = [0.1, 0.02, 0.01]
        self.spotlight_position_field.setSFVec3f(INITIAL_TRANS)
        self.spotlight_node.resetPhysics()
    
        # Evaluation genotype 
        self.run_seconds(self.time_experiment)  
        
        # Measure fitness
        fitness = self.receivedFitness
        print("Fitness without reward or penalty: {}".format(fitness))    
    
    def run_optimization(self):
        # Wait until the number of weights is updated
        while(self.num_weights == 0):
            self.handle_receiver()
            self.createRandomPopulation()
        
        print(">>>Starting Evolution using GA optimization ...\n")
        
        # For each Generation
        for generation in range(self.num_generations):
            print("Generation: {}".format(generation))
            current_population = []   
            # Select each Genotype or Individual
            for population in range(self.num_population):
                genotype = self.population[population]
                # Evaluate
                fitness = self.evaluate_genotype(genotype,generation)
                #print(fitness)
                # Save its fitness value
                current_population.append((genotype,float(fitness)))
                #print(current_population)
                
            # After checking the fitness value of all indivuals
            # Save genotype of the best individual
            best = ga.getBestGenotype(current_population);
            average = ga.getAverageGenotype(current_population);
            np.save("Best.npy",best[0])
            self.plot_fitness(generation, best[1], average);
            
            # Generate the new population using genetic operators
            if (generation < self.num_generations - 1):
                self.population = ga.population_reproduce(current_population,self.num_elite);
        
        #print("All Genotypes: {}".format(self.genotypes))
        print("GA optimization terminated.\n")   
    
    
    def draw_scaled_line(self, generation, y1, y2): 
        # the scale of the fitness plot
        XSCALE = int(self.width/self.num_generations);
        YSCALE = 100;
        self.display.drawLine((generation-1)*XSCALE, self.height-int(y1*YSCALE), generation*XSCALE, self.height-int(y2*YSCALE));
    
    def plot_fitness(self, generation, best_fitness, average_fitness):
        if (generation > 0):
            self.display.setColor(0xff0000);  # red
            self.draw_scaled_line(generation, self.prev_best_fitness, best_fitness);
    
            self.display.setColor(0x00ff00);  # green
            self.draw_scaled_line(generation, self.prev_average_fitness, average_fitness);
    
        self.prev_best_fitness = best_fitness;
        self.prev_average_fitness = average_fitness;
  
    
if __name__ == "__main__":
    # Call Supervisor function to initiate the supervisor module   
    gaModel = SupervisorGA()
    
    # Function used to run the best individual or the GA
    keyboard = Keyboard()
    keyboard.enable(50)
    
    # Interface
    print("***************************************************************************************************")
    print("To start the simulation please click anywhere in the SIMULATION WINDOW(3D Window) and press either:")
    print("(S|s)to Search for New Best Individual OR (R|r) to Run Best Individual")
    print("***************************************************************************************************")
    
    while gaModel.supervisor.step(gaModel.time_step) != -1:
        resp = keyboard.getKey()
        if(resp == 83 or resp == 65619):
            gaModel.run_optimization()
            print("(S|s)to Search for New Best Individual OR (R|r) to Run Best Individual")
            #print("(R|r)un Best Individual or (S|s)earch for New Best Individual:")
        elif(resp == 82 or resp == 65619):
            gaModel.run_demo()
            print("(S|s)to Search for New Best Individual OR (R|r) to Run Best Individual")
            #print("(R|r)un Best Individual or (S|s)earch for New Best Individual:")
        
