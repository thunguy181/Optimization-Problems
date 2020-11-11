# -*- coding: utf-8 -*-
"""
Created on Tue Nov 10 10:30:01 2020

@author: thu.nguyen
"""
#Intro: Recently I've been going to a lot of open houses and each house has a different open time. I also spend about 30 mins at each house. This is a working code to optimize my every weekend house hunting 
#Thanks to google API and OR-tools
#This code was ran using Spyder (installed using Anaconda)
#(1) First time running code, run the line below in Anaconda Navigator to installe OR-Tools
#       python -m pip install --upgrade --user ortools
#To build and run OR-Tools on Windows, you must have Visual Studio 2019 or later installed on your computer with the C++ toolset for Visual Studio 

#(2) Need to register API_key with Google and enable Distance Matrix API 

#(3) Import all required library
    #these library are for obtaining distance matrix from Google API
import requests
import json
from urllib.request import urlopen
import numpy as np

    #these library are for getting the optimal routes
from __future__ import print_function
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
#(4) Create time matrix (time taken to move from one house location to another using Google Distnace Matrix API)
#This function creates data input to build time matrix
def create_data():
  """Creates the data."""
  data = {}
  #Input API_key and Addresse of Open Houses. 
  data['API_key'] = 'Enter your API_key'
  data['addresses'] = ['30+Maryland+Ave+Rockville+MD', # depot
                       '5204+Russett+Rd+Rockville+MD',
                       '314+Lanark+Way+Silver+Spring+MD',
                       '6503+4th+Ave+Takoma+Park+MD',
                       '7+Rollins+Ct+Rockville+MD',
                       '9809+Capitol+View+Ave+Silver+Spring+MD',
                       '3501+Tavenner+Ct+Olney+MD'
                  ]
  return data

def create_distance_matrix(data):
  addresses = data["addresses"]
  API_key = data["API_key"]
  # Distance Matrix API only accepts 100 elements per request, so this function split rows in multiple requests.
  max_elements = 100
  num_addresses = len(addresses) # this example does not require splitting rows as there are only 7 addresses (7x7=49 rows).
  # Maximum number of rows that can be computed per request (6 in this example).
  max_rows = max_elements // num_addresses
  # num_addresses = q * max_rows + r.
  q, r = divmod(num_addresses, max_rows)
  dest_addresses = addresses
  distance_matrix = []
  # Send q requests, returning max_rows rows per request.
  for i in range(q):
    origin_addresses = addresses[i * max_rows: (i + 1) * max_rows]
    response = send_request(origin_addresses, dest_addresses, API_key)
    distance_matrix += build_time_matrix(response)

  # Get the remaining remaining r rows, if necessary (only use if max_elements is not divisible by num_addresses)
  if r > 0:
    origin_addresses = addresses[q * max_rows: q * max_rows + r]
    response = send_request(origin_addresses, dest_addresses, API_key)
    distance_matrix += build_time_matrix(response)
  return distance_matrix

def send_request(origin_addresses, dest_addresses, API_key):
  """ Build and send request for the given origin and destination addresses."""
  def build_address_str(addresses):
    # Build a pipe-separated string of addresses
    address_str = ''
    for i in range(len(addresses) - 1):
      address_str += addresses[i] + '|'
    address_str += addresses[-1]
    return address_str
    
#json at the end of the request ask for response in json; units=imperial sets the language of the response to English.
  request = 'https://maps.googleapis.com/maps/api/distancematrix/json?units=imperial'
  origin_address_str = build_address_str(origin_addresses)
  dest_address_str = build_address_str(dest_addresses)
  request = request + '&origins=' + origin_address_str + '&destinations=' + \
                       dest_address_str + '&key=' + API_key
  #urlopen is the right function for Python 3
  jsonResult = urlopen(request).read()
  response = json.loads(jsonResult)
  return response

def build_time_matrix(response):
  distance_matrix = []
  #iterate through the json to obtain 'Duration' value (time taken to travel from one point to another in second)
  for row in response['rows']:
    row_list = [row['elements'][j]['duration']['value'] for j in range(len(row['elements']))]
    distance_matrix.append(row_list)
  return distance_matrix

########
# Main #
########
def main():
  """Entry point of the program"""
  # Create the data.
  data = create_data()
  addresses = data['addresses']
  API_key = data['API_key']
  distance_matrix = create_distance_matrix(data)
  return distance_matrix
time_matrix = {}
if __name__ == '__main__':
    #Save in a matrix
    time_matrix = main()
    
#Convert time matrix to minutes because second is too much and add 10,20,30 minutes of house viewing to each record. Has to round up because the model only take integer
time_matrix2 = np.round(np.asarray(time_matrix)/60 +20,0)    

#(5) Create optimized route. As each house has a different opening time, use """Vehicles Routing Problem (VRP) with Time Windows."""

def create_data_model():
    """Stores the data for the problem."""
    data = {}
    data['time_matrix'] = time_matrix2
    #time_window is the window of time the house is open and needs to be visit; Since open houses are often in the afternoon, I'll leave at 12pm
    #Below is the number of minutes from 12pm
    data['time_windows'] = [
        (0, 60),  # depot: depart between 12pm and 1pm
        (120, 240), #First house open from 2-4pm
        (60, 240),  # 1-4pm
        (60, 180),  # 1-3pm    
        (60, 180),  # 1-3pm
        (60, 240),  # 1-4pm
        (120, 240),  # 2-4pm
    ]
    data['num_vehicles'] = 1
    data['depot'] = 0
    return data


def print_solution(data, manager, routing, solution):
    """Print solution to array"""
    time_dimension = routing.GetDimensionOrDie('Time')
    total_time = 0
    #Loop through each vehicle. In this example, only 1 vehicle so 1 loop
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        plan_output = 'Route for vehicle {}:\n'.format(vehicle_id)
        while not routing.IsEnd(index):
            #time_var = variable to keep track of cumulative time traveled
            time_var = time_dimension.CumulVar(index)
            plan_output += '{0} Time({1},{2}) -> '.format(
                manager.IndexToNode(index), solution.Min(time_var),
                solution.Max(time_var))
            index = solution.Value(routing.NextVar(index))
        time_var = time_dimension.CumulVar(index)
        #output the solution window (window for arrival and leave to be on schedule - this will need to be adjust as 30 mins was added)
        plan_output += '{0} Time({1},{2})\n'.format(manager.IndexToNode(index),
                                                    solution.Min(time_var),
                                                    solution.Max(time_var))
        #Total time traveled of the route
        plan_output += 'Time of the route: {}min\n'.format(
            solution.Min(time_var))
        print(plan_output)
        total_time += solution.Min(time_var)
    print('Total time of all routes: {}min'.format(total_time))


def main():
    """Solve the VRP with time windows."""
    # Instantiate the data problem.
    data = create_data_model()

    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(len(data['time_matrix']),
                                           data['num_vehicles'], data['depot'])

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)


    # Create and register a transit callback.
    def time_callback(from_index, to_index):
        """Returns the travel time between the two nodes."""
        # Convert from routing variable Index to time matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['time_matrix'][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(time_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Time Windows constraint.
    time = 'Time'
    routing.AddDimension(
        transit_callback_index,
        30000,  # allow infinite waiting time (not relevant to this problem)
        30000,  # allow infinite maximum time per vehicle (not relevant)
        False,  # Don't force start cumul to zero(vehicle can start at anytime, not just at 0 equivalent 12pm).
        time)
    time_dimension = routing.GetDimensionOrDie(time)
    # Add time window constraints for each location except depot.
    for location_idx, time_window in enumerate(data['time_windows']):
        if location_idx == 0:
            continue
        index = manager.NodeToIndex(location_idx)
        time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])
    # Add time window constraints for each vehicle start node.
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        time_dimension.CumulVar(index).SetRange(data['time_windows'][0][0],
                                                data['time_windows'][0][1])

    # Instantiate route start and end times to produce feasible times.
    for i in range(data['num_vehicles']):
        routing.AddVariableMinimizedByFinalizer(
            time_dimension.CumulVar(routing.Start(i)))
        routing.AddVariableMinimizedByFinalizer(
            time_dimension.CumulVar(routing.End(i)))

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    # Print solution on console.
    if solution:
        print_solution(data, manager, routing, solution)
    else:
        print("No solution found!")
        

if __name__ == '__main__':
    main()
#Result: If I stay 30 mins per house, no feasible solution is found
    #if I stay 20 mins per house, total time will be 272 mins and the route will be as follow
    #Route for vehicle 0:
#0 Time(0,0) -> 4 Time(60,60) -> 2 Time(96,96) -> 3 Time(130,130) -> 5 Time(170,170) -> 1 Time(206,206) -> 6 Time(236,236) -> 0 Time(272,272)
#Time of the route: 272min

