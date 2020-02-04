from threading import Thread
#from multiprocessing import Queue,Value
from queue_mac import Queue
import numpy as np
import time
import copy

class Automata(Thread):
    def __init__(self,function_list,verbose=False):
        super(Automata,self).__init__()
        
        self._controllable_events = []

        self._map = {}
        self._map_controllable_to_function(function_list)
        
        self._event_queue = Queue()
        
        self._index = 0
        self._exit = 0
        
        self._set_automata = False
        self._sync_flag = False
        self._verbose = verbose

    def has_automata(self):
        return self._set_automata
    
    def run(self):
        while (not self._exit):
            #Stops when the only thing left to do is wait for the environment
            self._generate_controllables()
            if self._exit:
                break
            
            #Wait for the environment
            event = self._event_queue.get()
            self._process_event(event)
    
    def get_event_queue(self):
        return self._event_queue
   
    def add_to_event_queue(self,event):
        self._event_queue.put(event)
        
    def _map_controllable_to_function(self,function_list):
        for function in function_list:
            fn_name = function.__name__
            self._controllable_events.append(fn_name)
            self._map[fn_name] = function
    
    def _generate_controllables(self):
        controllable_found = 1
        states = self._states
        while (controllable_found):
            while(self._event_queue.qsize() > 0):
                event = self._event_queue.get()
                self._process_event(event)
                if (self._exit):
                    return
            
            #Search for controllables
            controllable_found = 0
            for elem in states[self._index]:
                if (elem[1] == 'C'):
                    controllable_found = 1
                    self._process_event(elem[0],controllable=True)
                    break
        return
    
    def _process_event(self,event,controllable=False):
        self._event2 = event
        if (self._verbose):
            print("Event: " + event)
        if (event == 'exit'):
            self._exit = 1
            return
        
        for elem in self._states[self._index]:
            if(elem[0] == event):
                self._index = int(elem[2])
                
                if controllable:
                    try:
                        values = elem[0].split('[')
                        if (len(values) == 1):
                            self._map[elem[0]]()
                        else:
                            for i in range(1,len(values)):
                                values[i] = values[i][:-1]
                            args = tuple(values[1:])
                            self._map[values[0]](*args)
                            
                    except KeyError:
                        print(elem[0] + " action not callable")
                        
                return
            
        print("EVENT " + event + " NOT FOUND")
        self.add_to_event_queue('exit')
                

    def load_automata_from_file(self,filename):
        file = open(filename,'r')
        automata_data = file.read()
        file.close()

        self.load_automata(automata_data)
    
    def load_automata(self,automata_data):
        lines = automata_data.splitlines()
        states_size = int(lines[3].strip())
        states = [[]]*states_size
        
        analysing_state = False
        num_state = 0
        for i in range(6,len(lines)):
            line = lines[i].strip()
            
            if(analysing_state == False):
                if (line[0] == 'Q'):
                    analysing_state = True
                    aux_vec = []
            if(analysing_state == True):
                    aux_str = ''
                    many_events = False
                    action_set = False
                    for char in line:
                        if (char == '='):
                            num_state = int(aux_str[1:].strip())
                            continue
                        
                        if (char in ['(','{','|']):
                            aux_str = ''
                            continue
                        
                        if (char == '}'):
                            many_events = True
                            action = aux_str
                            action_set = True
                            continue
                        
                        if (many_events == False and char == '-'):
                            action = aux_str
                            action_set = True
                            continue
                        
                        if (action_set == True):
                            if (char == 'Q'):
                                aux_str = ''
                                continue
                        
                        if (char == ')'):
                            num_next_state = int(aux_str.strip())
                            analysing_state = False
                            break
                    
                        aux_str = aux_str + char
                    
                    if(analysing_state == True):
                        num_next_state = int(aux_str.strip())
                        
                    if (many_events == True):
                        aux_lista = np.array([])
                        aux_str = ''
                        for j in range(len(action)):
                            if (action[j] == ','):
                                aux_lista = np.append(aux_lista,aux_str)
                                aux_str = ''
                                continue
                            aux_str += action[j]
                            if (j == len(action)-1):
                                aux_lista = np.append(aux_lista,aux_str)
                        action = aux_lista
                    else:
                        action = [action]
                    
                    for accion in action:
                        accion = accion.strip()
                        values = accion.split('[')
                        if (values[0] in self._controllable_events):
                            aux_var = 'C'
                        else:
                            aux_var = 'A'
                        aux_vec.append([accion,aux_var,num_next_state])
                    
                    if(analysing_state == False):
                        states[num_state] = aux_vec

        self._states = states
        self._set_automata = True
        if (self._verbose):
            print("Automata loaded")
        '''
        self._states = np.array([
            [['accion','C',1]], #0
            [['evento','A',0]] #1
            
        ])
        '''
        return
