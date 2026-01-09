import numpy as np
"""
Smallest unit is gen
Unit : gen --> chromosome --> individual --> population
6 gen = 1 chromosome
2 chromosome = 1 individual
(n) individual = 1 population [n is a number ]
"""
"0 = go ahead "
"1 = right"
"2 = left"
class Genetic_Algo:
    def __init__(self, ranged_decode_X, ranged_decode_Y, population_size, num_chromo, num_gene):
        self.ranged_decode_X = ranged_decode_X
        self.ranged_decode_Y = ranged_decode_Y
        self.population_size = population_size
        self.num_chromo = num_chromo
        self.num_gene = num_gene
        

    # Step 1 : Create population
    def create_population(self):
        population = np.random.randint(0,10,size=(self.population_size, self.num_chromo, self.num_gene))
        #self.population = population
        return population
    

    # Step 2 : Decode population
    def decode_gen(self, population):
        chromosome = 0
        gen_max = 10**(self.num_gene) - 1
        gen_min = 000000

        # Population after decode
        decode_individual = []
        decode_population = []
        #self.decode_population = decode_population
        
        for individual in range(self.population_size):
            #print(individual)
            #print(pop[individual])
            for chrome in range(self.num_chromo):
                for gene in range(self.num_gene):
                    #print(pop[individual,chrome,gene])
                    gen = population[individual,chrome,gene] # Gen still in array type
                    gen_convert = gen * pow(10, (self.num_gene - gene -1)) # Convert array to number  
                
                    chromosome = gen_convert + chromosome # sum all gen
                    if (gene == (self.num_gene - 1)) and ((chrome + 1) % self.num_chromo != 0): # X value--> First chromosome is X value and the second is Y value
                        #print(chromosome)
                        #replace decode formula this line and remove this comment
                        chromosome = (((chromosome - gen_min) * (self.ranged_decode_X[1] - self.ranged_decode_X[0])) / (gen_max - gen_min)) + self.ranged_decode_X[0]
                        decode_individual.append(chromosome) # 2 decode chromosome = 1 decode individual
                        chromosome = 0


                    if (gene == (self.num_gene - 1)) and ((chrome + 1) % self.num_chromo == 0):
                        #print(chromosome)
                        #replace decode formula this line and remove this comment
                        chromosome = (((chromosome - gen_min) * (self.ranged_decode_Y[1] - self.ranged_decode_Y[0])) / (gen_max - gen_min)) + self.ranged_decode_Y[0]
                        decode_individual.append(chromosome) # 2 decode chromosome = 1 decode individual
                        chromosome = 0                        
        
                        # Add decode individuals to the population after decoding 2 chromosomes. 
                        decode_population.append(decode_individual)
                        decode_individual = []


        decode_population = np.array(decode_population)                
        return decode_population
   

    # Step 3 : Fitness value of each individual ( based on decode population )
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

    
    
    # Step 4 : Selection new population
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
        rest_of_newpop = np.random.randint(0,9,size=(self.population_size - len(new_population), self.num_chromo, self.num_gene))
        chosen_population = np.vstack((new_population, rest_of_newpop)) # Last population
        #self.chosen_population = chosen_population
        #print(chosen_population, len(chosen_population))
        return chosen_population
        
    # Step 5 : Cross over new population
    def crossover(self, chosen_population):
        # Create where to split chromosome
        crossing_point = np.random.randint(1, self.num_gene - 1)


        # Select individual for crossover
        """
        In this example individual with even index will be crossover with individual with odd index
        For example 0 and 2 will be crossover with 1 and 3
        """
        individual_even = chosen_population[0::2,:,:] # Take the even individual of population -- 0::2 mean start from inv zero, step 2. Ex: 0, 2, 4
        individual_odd = chosen_population[1::2,:,:] # Take the odd individual of individual  -- 1::2 start from inv 1, step 2. Ex :1, 3, 55
    
        # Test
        #print("This is all odd individual {}".format(individual_odd))
        #print("This is all even individual {}".format(individual_even))
    
    
        """
        columns = cot
        row = hang
        """
        Half = np.dstack((individual_even[:,:,0:crossing_point],individual_odd[:,:,crossing_point:])) # This is half of chromosome of one individual
        Other_half= np.dstack((individual_odd[:,:,0:crossing_point],individual_even[:,:,crossing_point:])) # This is the other half of chromosome of one individual

        cross_population = np.vstack((Half,Other_half))
        #self.cross_populaton = cross_population
        #print("This is new population after crossover: \n {}".format(new_population))
        return cross_population
        """
        Example this is one individual named P
        P = [   [1 3 5 8 9] ----> This is 1st chromosome
                [2 3 5 9 10] ]------> This is 2nd chromosome 

        """
    
    
    # Step 6 : mutation
    def mutation(self,cross_population, mutation_rate):
        population = cross_population
        for individual in range(self.population_size):
            for chromo in range(self.num_chromo):
                if (np.random.rand() < mutation_rate):
                    indices_to_change = np.random.choice(self.num_gene, size=1, replace=False)
                    for gen in (indices_to_change):
                        population[individual, chromo, gen] = np.random.randint(0, 10)
                        #print("Individual: {}, chromo: {}, gen: {}, change: {}".format(individual, chromo, gen, population[individual,chromo,gen]))
        
        return population





