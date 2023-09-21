
import json
import matplotlib.pyplot as plt
import time


"""

Use dictionary


rocket_design 
  ballast_top: float, 
  ballast_tottom: float


rocket_path 
  apogee : float, 
  stability : float, 
  absolute_distance_from_launch : float


"""




IDEAL_SCORE = 0
def score_rocket_path(rocket_path):
  return  rocket_path['apogee'] - 30000     # return offset from target


def iterate(rocket_path, rocket_design):

  # TODO
  return rocket_design


def simulate_rocket(rocket_design):
  #TODO interface with rocket py 
  time.sleep(3)
  return {'apogee': 29000, 'stability': 1.2}



"""
# use to load automatically
(rocket_scores, rocket_designs) = json.loads("")
rocket_design = rocket_designs[-1]
"""





# default
(rocket_scores, rocket_designs) = ([], [])
rocket_design = {} 


def graph():

  # Create x-axis values as indexes (0, 1, 2, ...)

  rng = range(len(rocket_scores))

  plt.plot(rng, rocket_scores, label='rocket_scores')

  # TODO rocket_designs
  #plt.plot(rng, b, label='List B')


  # Add labels and a legend
  plt.xlabel('Index')
  plt.ylabel('Values')
  plt.legend()

  # Save the plot as an image (e.g., PNG format)
  plt.savefig('my_plot.png')
  plt.close()



while True:

  rocket_path = simulate_rocket(rocket_design)
  rocket_design = iterate(rocket_path, rocket_design)
  score = score_rocket_path(rocket_path)

  rocket_scores.append(score)
  rocket_designs.append(rocket_design)

  graph() 

  print(json.dumps([rocket_scores, rocket_designs]))  # just use strings, files are overkill


