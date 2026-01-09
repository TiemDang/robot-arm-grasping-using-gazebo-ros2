import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String
import time
import math
import numpy as np
from cli_spawn.class_GA import Genetic_Algo
from cli_spawn.ClassNeuralNetwork import NeuralNet
import matplotlib.pyplot as plt
import pickle

class GA_calc(Node):
    def __init__(self):
        super().__init__('ga_calculate')
        # Publish and subscribe
        self.state_pub = self.create_publisher(String, 'state_check', 10)
        self.pose_sub = self.create_subscription(Float64MultiArray, 'endpoint_pose', self.get_pose, 10)
        self.ga_pub = self.create_publisher(Float64MultiArray, 'ga_population', 10)
        self.generation_sub = self.create_subscription(String, 'state_check', self.generation_callback, 10)
        
        # Directory to save model
        self.save_dir = "/home/venus/ros2_ws/src/cli_spawn/models/"
        # position & orientation default (start position of the gripper on robot)
        self.x_default = None
        self.y_default = None
        self.z_default = None
        self.ox_default = None
        self.oy_default = None
        self.oz_default = None
        self.ow_default = None
        # pos & ori current
        self.x_current = None
        self.y_current = None
        self.z_current = None
        self.ox_current = None
        self.oy_current = None
        self.oz_current = None
        self.ow_current = None
        
        # pos & ori target
        self.x_target = -0.03
        self.y_target = -0.081
        self.z_target = 0.641
        self.ox_target = 0.705
        self.oy_target = 0.503
        self.oz_target = 0.705
        self.ow_target = 0.503

        # GA parameter
        self.population_size = 80
        self.generation = 150
        self.current_generation = 1
        
        
        # save population
        self.min_fitness_history = []
        self.current_individual_index = 0 
        self.final_theta = None
        self.final_fitness = 99999
        self.save_raw_gene = None
        
        # GA memo
        self.GA_init = Genetic_Algo([-1, 1],[-1, 1], self.population_size, 756, 3)
        self.raw_population = self.GA_init.create_population() # create generation 1
        self.decode_population = self.GA_init.decode_gen(self.raw_population) # decode generation 1
        self.theta_population = [] # result theta
        self.error_log = [] # fitness

    #----------------------------------------------------------
    def update_state(self,msg):
        state_msg = String()
        state_msg.data = msg
        self.state_pub.publish(state_msg)
        self.get_logger().info(f'State published: {msg}')
        
    #----------------------------------------------------------
    def save_model(self, theta, filename):
        w = np.reshape(theta[:588], (14, 42))
        v = np.reshape(theta[588:], (42, 4))
        model_data = {"w": w, "v": v}
        
        with open(filename, "wb") as f:
            pickle.dump(model_data, f)
        print(f"Model saved at {filename}")
    
    #----------------------------------------------------------
    def fitness_cal(self):
        fitness = math.sqrt((self.x_current - self.x_target)**2 + (self.y_current - self.y_target)**2 + (self.z_current - self.z_target)**2 + (self.ox_current - self.ox_target)**2 + (self.oy_current - self.oy_target)**2
                             + (self.oz_current - self.oz_target)**2 + (self.ow_current - self.ow_target)**2)
        return fitness
    
    
    #----------------------------------------------------------
    def get_pose(self, pose_msg):
        x_current, y_current, z_current, ox_current, oy_current, oz_current, ow_current = pose_msg.data[0], pose_msg.data[1], pose_msg.data[2], pose_msg.data[3], pose_msg.data[4], pose_msg.data[5], pose_msg.data[6]
        if self.x_default is None and self.ox_default is None:
            # Get default position
            self.x_default = x_current
            self.y_default = y_current
            self.z_default = z_current
            self.ox_default = ox_current
            self.oy_default = oy_current
            self.oz_default = oz_current
            self.ow_default = ow_current
            # create population
            self.ga_publisher()
            self.get_logger().info(f"Default pos & ori: x = {self.x_default}, y = {self.y_default}, z = {self.z_default}, ox = {self.ox_default}, oy = {self.oy_default}, oz = {self.oz_default}, ow = {self.ow_default}")
            self.update_state("Calculating_Done")
        else :
            self.x_current = x_current
            self.y_current = y_current
            self.z_current = z_current
            self.ox_current = ox_current
            self.oy_current = oy_current
            self.oz_current = oz_current
            self.ow_current = ow_current
            individual_error = self.fitness_cal()
            self.error_log.append(individual_error)
            #self.xy_log.append((x_current, y_current))
            
            # Format to print current individual info : index, error, theta
            theta_current = self.theta_population[len(self.error_log)-1]
            theta_format = [f"{t:.2f}" for t in theta_current]
            print(f"Individual {len(self.error_log)}: theta = {theta_format}, error = {individual_error:.3f}")
            if len(self.error_log) == self.population_size :
                print("Complete one geneneration, begin calculating...")
                
                # Index
                min_fitness = min(self.error_log)
                min_index = self.error_log.index(min_fitness)
                best_raw_gene = self.decode_population[min_index]
                best_theta = self.theta_population[min_index]
                self.min_fitness_history.append(min_fitness)
                
                # Save model
                if min_fitness < self.final_fitness :
                    self.final_fitness = min_fitness
                    self.final_theta = best_theta
                    self.save_raw_gene = best_raw_gene
                if ((self.current_generation + 1) % 15) == 0 :
                    model_name = "model_" + str(self.current_generation + 1) + ".pkl"
                    save_path = self.save_dir + model_name
                    self.save_model(self.save_raw_gene, save_path)
                
                # Result of generation
                print(f"Calculating gen {self.current_generation} done. Min fitness of generation : {min_fitness}, Best theta of generation : {self.theta_population[min_index]}, "
                      f"Best theta : {self.final_theta}, Best fitness : {self.final_fitness}"
                )
                # Starting new gen
                selection_pop = self.GA_init.selection(self.error_log, self.raw_population)
                cross_over_pop = self.GA_init.crossover(selection_pop)
                self.raw_population = self.GA_init.mutation(cross_over_pop, 0.1)
                self.decode_population = self.GA_init.decode_gen(self.raw_population) # decode will go to ga_publisher > convert to theta_pop then publish
                self.error_log = [] # reset error log

                # Update state
                self.current_generation += 1
                self.ga_publisher()
                self.update_state("Calculating_All_Gene_Done")
            else :    
                self.update_state("Calculating_Done")
            #self.get_logger().info(f"Received : x = {self.x_current:.3f}, y = {self.y_current:.3f}")
            
    
    #----------------------------------------------------------
    def generation_callback(self, msg):
        if msg.data == "Calculating_All_Gene_Done":
            if self.current_generation <= self.generation :
                pass
                #self.current_generation += 1
                #self.get_logger().info(f"---> Starting Generation {self.current_generation}")
                #self.ga_publisher()
            else :
                self.get_logger().info("---> All generations completed.")
                self.update_state("Generations Done")
                self.plot_fitness()
   
   
   #----------------------------------------------------------
    def ga_publisher(self):
        # Generation var
        generation = self.current_generation
        new_theta_population = []
        
        self.get_logger().info(f"Generation {generation}/{self.generation}...")
        for idx, individual in enumerate(self.decode_population):
            w = np.reshape(individual[:588], (14, 42))
            v = np.reshape(individual[588:], (42, 4))
            x = np.array([self.x_default, self.y_default, self.z_default, self.ox_default, self.oy_default, self.oz_default, self.ow_default,
                         self.x_target, self.y_target, self.z_target, self.ox_target, self.oy_target, self.oz_target, self.ow_target]).reshape(14,1)
            output = NeuralNet(x, w, v).FeedForward()
            # Create individual
            theta_individual = [
                round(float(output[0][0]), 3),
                round(float(output[1][0]), 3),
                round(float(output[2][0]), 3),
                round(float(output[3][0]), 3),
                0.0,0.03,-0.03
            ]
            new_theta_population.append(theta_individual)
        #print(new_theta_population)
        self.theta_population = new_theta_population
            
        # Publish 
        ga_msg = Float64MultiArray()
        ga_msg.data = np.array(self.theta_population).flatten().tolist()
        self.ga_pub.publish(ga_msg)
        self.get_logger().info(f"Published population with {len(self.theta_population)} individuals.")
        
    
    #----------------------------------------------------------
    def plot_fitness(self):
        if not self.min_fitness_history:
            self.get_logger().warn("No fitness history recorded.")
            return

        generations = list(range(1, len(self.min_fitness_history) + 1))
        plt.figure(figsize=(6, 5))
        plt.plot(generations, self.min_fitness_history, 'o-', label='Min fitness per generation')
        plt.xlabel('Generation')
        plt.ylabel('Min Fitness')
        plt.title('Fitness Progress over Generations')
        plt.grid(True)
        plt.legend()
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    node = GA_calc()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

