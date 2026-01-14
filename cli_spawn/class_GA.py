import numpy as np
"""
Smallest unit is chromosome
Unit : chromosome --> individual --> population
2 chromosome = 1 individual
(n) individual = 1 population [n is a number ]
"""

class Genetic_Algo:
    def __init__(self, min, max, population_size, num_chromo):
        self.min = min
        self.max = max
        self.population_size = population_size
        self.num_chromo = num_chromo
        

    # Step 1 : Create population
    def create_population(self):
        population = np.random.uniform( self.min, self.max,size=(self.population_size, self.num_chromo))
        #self.population = population
        return population
    

    # Step 2 : Fitness value of each individual
    def fitness(self, decode_population):
        fitness_list = []
        #self.fitness_list = fitness_list
        
        for i in range(len(decode_population)):
            chromo1 = decode_population[i][0]
            chromo2 = decode_population[i][1]
            #print("Chromo1 : {}, Chromo2 : {}".format(chromo1, chromo2))
            """
            Test fitness function
            #fitness = -20*np.exp(-0.2*np.sqrt(0.5*(chromo1**2 + chromo2**2))) - np.exp(0.5*(np.cos(2*np.pi*chromo1)+ np.cos(2*np.pi*chromo2))) + 20 + np.exp(1)
            #fitness = (chromo1**2 + chromo2 -11)**2 + (chromo1 + chromo2**2 -7)**2
            """
            fitness = 4*pow(chromo1,2) - 2.1*pow(chromo1,4) + 1/3*pow(chromo1,6) + chromo1*chromo2 - 4*pow(chromo2,2) + 4*pow(chromo2,4) # This function have condition --> add penelty 
            
            # Penalty
            if ((-np.sin(4*np.pi*chromo1) + 2*np.sin(2*np.pi*chromo2)**2) > 1.5) :
                fitness = fitness + 1000
            
            
            fitness_list.append(fitness)        
        return fitness_list

    
    
    # Step 3 : Selection new population
    def selection(self, fitness_list, population):
        new_population = [] # Save chosen individual from old population
        fitness_sum = 0
        fitness_average = 0

        
        # Test selection method : just take 1 individual with min fitness
        min_index = fitness_list.index(min(fitness_list))
        new_population = np.array([population[min_index]])

        """
        for index in range(len(fitness_list)):
            fitness_sum = fitness_sum + fitness_list[index]
        fitness_average = fitness_sum / len(fitness_list)
        for id in range(len(fitness_list)):
            if (fitness_list[id] < fitness_average) :
                if (len(new_population)) == 0 :
                    new_population = np.array([population[id]])
                else :
                    add_individual = np.array([population[id]])
                    new_population = np.vstack((new_population,add_individual))
        """
        rest_of_newpop = np.random.uniform(self.min, self.max ,size=(self.population_size - len(new_population), self.num_chromo))
        chosen_population = np.vstack((new_population, rest_of_newpop)) # Last population
        #self.chosen_population = chosen_population
        #print(chosen_population, len(chosen_population))
        #print(f'Best individual after selectionn : {chosen_population[0]}')
        return chosen_population
        
    # Step 4 : Cross over new population
    def crossover(self, chosen_population):
        # Create where to split chromosome
        crossing_point = np.random.randint(1, self.num_chromo)


        # Select individual for crossover
        """
        In this example individual with even index will be crossover with individual with odd index
        For example 0 and 2 will be crossover with 1 and 3
        """
        individual_even = chosen_population[0::2, :] # Take the even individual of population
        individual_odd = chosen_population[1::2, :] # Take the odd individual of individual 
    
        # Test
        #print("This is all odd individual {}".format(individual_odd))
        #print("This is all even individual {}".format(individual_even))
    
    
        """
        columns = cot
        row = hang
        """
        child1 = np.hstack((individual_even[:, :crossing_point], individual_odd[:,  crossing_point:])) # This is half of chromosome of one individual
        child2 = np.hstack((individual_odd[:,  :crossing_point], individual_even[:, crossing_point:])) # This is the other half of chromosome of one individual

        cross_population = np.vstack((child1,child2))
        #self.cross_populaton = cross_population
        #print("This is new population after crossover: \n {}".format(new_population))
        #print(f'Best individual after crossover : {cross_population[0]}')
        return cross_population
        """
        Example this is one individual named P
        P = [   [1 3 5 8 9] ----> This is 1st chromosome
                [2 3 5 9 10] ]------> This is 2nd chromosome 

        """
    
    
    # Step 5 : mutation
    def mutation(self,cross_population, mutation_rate, sigma):
        pop, chr  = cross_population.shape
        # pop = total individual of population
        # chr = number chromosome 1 individual have
        for individual in range(pop) :
            mask = np.random.rand(chr) < mutation_rate
            # Ex individual = [1 2 1 0] and mutation rate = 0
            # mask = [ 1 < mutation_rate = 0 so F,
            #          2 < mutation_rate = 0 so F,
            #          1 < mutation rate = 0 so F,
            #          0 < mutation_rate = 0 so F]
            # mask = [F, F, F, F] mean no mutation
            # if mask = [T, F, F, T] mean mutation at chromosome 1 and 4
            mutation_value = np.random.normal(0, sigma, size=np.sum(mask))
            print(mutation_value)
            cross_population[individual, mask] += mutation_value
            population = cross_population
        return population



